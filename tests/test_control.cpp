#include "bezier.h"
#include "config.h"
#include "drive.h"
#include "ladrc2.h"
#include "odometry.h"
#include "scurve.h"
#include <cassert>
#include <cmath>
#include <iostream>

static bool near(double a, double b, double e) { return std::fabs(a - b) <= e; }
static void check(bool value, const char* message) {
    if (!value) { std::cerr << "FAIL: " << message << "\n"; std::abort(); }
}

// Proves first-order fitting and b0 recovery stay within 1% relative error;
// invalid tau and voltage must return the exact placeholder value 0.0.
static void test_identification()
{
    const double tau = 0.8, b0 = 2.5, u = 4.0, vss = b0 * tau * u;
    double t[81], v[81];
    for (int i = 0; i < 81; ++i) {
        t[i] = i * 0.05;
        v[i] = vss * (1.0 - std::exp(-t[i] / tau)) +
               0.002 * std::sin(13.0 * t[i]);
    }
    double fit_tau = 0.0, fit_vss = 0.0;
    check(FitStepResponse(t, v, 81, fit_tau, fit_vss), "step response fit");
    check(std::fabs(fit_tau - tau) / tau < 0.01, "tau within 1 percent");
    check(std::fabs(ComputeB0(fit_vss, fit_tau, u) - b0) / b0 < 0.01,
          "b0 within 1 percent");
    check(ComputeB0(1.0, 0.0, 2.0) == 0.0, "invalid tau rejected");
    check(ComputeB0(1.0, 1.0, 0.0) == 0.0, "zero voltage rejected");
}

// Proves b0 must be positive and finite, acceleration headroom must be <=70%,
// and the jerk heuristic must satisfy jmax/amax <= wc.
static void test_safety()
{
    check(!ValidateScurveLimits(10.0, 10.0, 0.0, 4.0, 12.0), "zero b0 rejected");
    check(!ValidateScurveLimits(10.0, 10.0, -1.0, 4.0, 12.0), "negative b0 rejected");
    check(ValidateScurveLimits(50.0, 100.0, 10.0, 4.0, 12.0), "valid limits accepted");
    check(!ValidateScurveLimits(100.0, 100.0, 1.0, 4.0, 12.0), "headroom failure rejected");
    check(!ValidateScurveLimits(10.0, 100.0, 10.0, 4.0, 12.0), "jerk heuristic rejected");
}

// Proves endpoints and final heading are within 1e-9, curvature is within
// 1e-6 of the 1/R limit, and an infeasible radius is rejected.
static void test_bezier()
{
    BezierPath path;
    check(path.Configure(600.0, 300.0, 0.4, 300.0), "bezier configuration");
    check(near(path.PointAt(0.0).x, 0.0, 1e-9) &&
          near(path.PointAt(0.0).y, 0.0, 1e-9), "bezier start");
    Point2 end = path.PointAt(path.length());
    check(near(end.x, 600.0, 1e-9) && near(end.y, 300.0, 1e-9), "bezier end");
    check(near(path.HeadingAt(path.length()), 0.4, 1e-9), "bezier end heading");
    check(path.max_curvature() <= 1.0 / 300.0 + 1e-6, "curvature limit");
    check(path.HeadingAt(path.length()) == path.HeadingAt(path.length()),
          "finite end heading");
    BezierPath impossible;
    check(!impossible.Configure(1.0, 1000.0, 0.0, 0.001),
          "infeasible curvature reports failure");
}

// Proves analytic duration within 1e-9 or 1e-6, monotonic position within
// 1e-8, and velocity/acceleration limits within 1e-7.
static void test_scurve()
{
    ScurveProfile cruise;
    check(cruise.Configure(3000.0, 600.0, 300.0, 1200.0), "cruising profile");
    const double tj = 300.0 / 1200.0;
    const double ta = 600.0 / 300.0 - tj;
    const double half = 1200.0 * tj * tj * tj / 6.0 +
        (0.5 * 1200.0 * tj * tj) * ta +
        0.5 * (1200.0 * tj) * ta * ta +
        (0.5 * 1200.0 * tj * tj + 1200.0 * tj * ta) * tj +
        0.5 * (1200.0 * tj) * tj * tj - 1200.0 * tj * tj * tj / 6.0;
    const double expected = 2.0 * (2.0 * tj + ta) + (3000.0 - 2.0 * half) / 600.0;
    check(near(cruise.duration(), expected, 1e-9), "cruising analytic duration");
    double last = 0.0;
    for (int i = 0; i <= 2000; ++i) {
        ScurveState q = cruise.Sample(cruise.duration() * i / 2000.0);
        check(q.position >= last - 1e-8, "position monotonic");
        check(std::fabs(q.velocity) <= 600.0 + 1e-7, "velocity limit");
        check(std::fabs(q.acceleration) <= 300.0 + 1e-7, "acceleration limit");
        last = q.position;
    }
    ScurveProfile triangular;
    check(triangular.Configure(20.0, 600.0, 300.0, 1200.0), "triangular profile");
    const double tri_tj = std::pow(20.0 / (2.0 * 1200.0), 1.0 / 3.0);
    check(near(triangular.duration(), 4.0 * tri_tj, 1e-6), "triangular analytic duration");
    ScurveState final = cruise.Sample(cruise.duration() + 1.0);
    check(near(final.position, 3000.0, 1e-9) && final.velocity == 0.0 &&
          final.acceleration == 0.0, "final hold");
}

// Proves disturbance tracking error stays below 15, output stays within 12 V,
// invalid data clears valid_, finite recovery restores it, and applied ESO
// input plus DriveController handoff agree within 1e-9.
static void test_ladrc()
{
    Ladrc2 controller(4.0, 100.0);
    controller.Reset(0.0);
    double y = 0.0, velocity = 0.0;
    double max_error = 0.0, max_output = 0.0;
    for (int i = 0; i < 6000; ++i) {
        const double disturbance = i < 2500 ? 0.0 : 20.0;
        const double output = controller.Update(0.001, y, 100.0, 0.0, 0.0);
        velocity += 0.001 * (disturbance + 100.0 * output);
        y += 0.001 * velocity;
        if (i > 4500) max_error = std::max(max_error, std::fabs(100.0 - y));
        max_output = std::max(max_output, std::fabs(output));
    }
    check(max_error < 15.0, "disturbance tracking bound");
    check(max_output <= 12.0 + 1e-9, "voltage saturation");
    Ladrc2 saturated(4.0, 0.01);
    saturated.Reset(0.0);
    const double first = saturated.Update(0.01, 0.0, 10000.0, 0.0, 0.0);
    const double second = saturated.Update(0.01, 0.0, 10000.0, 0.0, 0.0);
    check(first == 12.0 && second == 12.0, "saturated output remains bounded");

    Ladrc2 health(4.0, 10.0);
    health.Reset(0.0);
    health.Update(0.01, std::nan(""), 0.0, 0.0, 0.0);
    check(!health.valid(), "invalid sample marks controller unhealthy");
    health.Update(0.01, 0.0, 0.0, 0.0, 0.0);
    check(health.valid(), "finite sample restores controller health");

    Ladrc2 applied(4.0, 10.0);
    applied.Reset(0.0);
    applied.SetAppliedInput(2.4);
    applied.Update(0.01, 0.0, 0.0, 0.0, 0.0);
    check(std::fabs(applied.velocity_estimate() - 0.24) < 1e-9,
          "ESO uses applied scaled input");

    BezierPath mixed_path;
    check(mixed_path.Configure(1000.0, 0.0, 0.0, 0.0),
          "mixed saturation path");
    DriveController mixed_drive(4.0, 10.0, 4.0, 10.0);
    check(mixed_drive.StartPath(mixed_path, {0.0, 0.0, 0.0},
                                300.0, 20.0, 40.0),
          "mixed saturation start");
    const DriveVoltages first_mixed =
        mixed_drive.Update(0.01, {1000.0, 0.0, 1.0}, 12.0);
    mixed_drive.Update(0.01, {1000.0, 0.0, 1.0}, 12.0);
    Ladrc2 expected_longitudinal(4.0, 10.0);
    expected_longitudinal.Reset(0.0);
    expected_longitudinal.Update(0.01, 1000.0, 0.0, 0.0, 0.0);
    expected_longitudinal.SetAppliedInput(
        0.5 * (first_mixed.left + first_mixed.right));
    expected_longitudinal.Update(0.01, 1000.0, 0.0, 0.0, 0.0);
    check(std::fabs(mixed_drive.longitudinal_velocity_estimate() -
                    expected_longitudinal.velocity_estimate()) < 1e-9,
          "DriveController feeds scaled voltage to ESO");
}

// Proves finite forward odometry, expected 100 mm travel within 5 mm, and
// bit-identical pose preservation after a non-finite sensor sample.
static void test_odometry()
{
    Odometry odom(50.0);
    OdometryMeasurement m = {0, 0, 0, 0, 0, 0};
    odom.Reset(m, 0.0);
    m.left_mm = 100.0; m.right_mm = 100.0;
    m.horizontal_mm = 10.0; m.inertial_heading = 0.1;
    Pose2 pose = odom.Update(1.0, m);
    check(std::isfinite(pose.x) && std::isfinite(pose.y), "odometry finite");
    check(pose.x > 95.0 && pose.x < 105.0, "odometry forward motion");
    const Pose2 before_invalid = odom.pose();
    m.left_mm = std::nan("");
    const Pose2 after_invalid = odom.Update(1.0, m);
    check(after_invalid.x == before_invalid.x &&
          after_invalid.y == before_invalid.y &&
          after_invalid.heading == before_invalid.heading,
          "non-finite odometry leaves pose unchanged");
}

// Proves motor outputs stay within 12 V, invalid restart clears active(), and
// elapsed profile time freezes while the requested output is saturated.
static void test_drive_saturation()
{
    BezierPath path;
    check(path.Configure(1000.0, 0.0, 0.0, 0.0), "drive test path");
    DriveController drive(4.0, 100.0, 4.0, 10.0);
    const Pose2 start_pose = {10.0, 0.0, 0.05};
    check(drive.StartPath(path, start_pose, 300.0, 20.0, 40.0),
          "drive controller start");
    check(!drive.StartPath(path, start_pose, 300.0, 10000.0, 40.0),
          "invalid restart stops active controller");
    check(!drive.active(), "invalid restart clears active state");
    check(drive.StartPath(path, start_pose, 300.0, 20.0, 40.0),
          "drive controller restart");
    Pose2 far_away = {-1000.0, 500.0, 1.0};
    DriveVoltages output = drive.Update(0.01, far_away, 12.0);
    const double frozen = drive.elapsed();
    check(std::fabs(output.left) <= 12.0 && std::fabs(output.right) <= 12.0,
          "drive voltage saturation");
    drive.Update(0.01, far_away, 12.0);
    check(drive.elapsed() == frozen, "profile freezes during saturation");
}

// Proves sampled v^2*kappa never exceeds lateral_acceleration_max by more than
// 1e-6 mm/s^2.
static void test_curvature_speed_limit()
{
    BezierPath path;
    check(path.Configure(600.0, 300.0, 0.4, 300.0), "curvature-limited path");
    DriveController drive(4.0, 100.0, 4.0, 10.0);
    check(drive.StartPath(path, {0.0, 0.0, 0.0}, 600.0, 300.0, 1200.0),
          "curvature-limited start");
    const double kappa = path.max_curvature();
    double max_lateral_acceleration = 0.0;
    for (int i = 0; i <= 2000; ++i) {
        const double time = drive.profile().duration() * i / 2000.0;
        const ScurveState state = drive.profile().Sample(time);
        max_lateral_acceleration = std::max(
            max_lateral_acceleration, state.velocity * state.velocity * kappa);
    }
    check(max_lateral_acceleration <= config::lateral_acceleration_max + 1e-6,
          "curvature speed limit");
}

// Proves final hold remains active outside tolerance and for 0.19 s, then
// becomes inactive after the configured 0.20 s settle interval.
static void test_final_hold()
{
    BezierPath path;
    check(path.Configure(20.0, 0.0, 0.0, 0.0), "final hold path");
    DriveController drive(4.0, 100.0, 4.0, 10.0);
    check(drive.StartPath(path, {0.0, 0.0, 0.0}, 100.0, 100.0, 300.0),
          "final hold start");
    for (int i = 0; i < 1000 && drive.active() &&
                 drive.elapsed() < drive.profile().duration(); ++i) {
        const ScurveState state = drive.profile().Sample(drive.elapsed());
        const Point2 point = path.PointAt(state.position);
        drive.Update(0.01, {point.x, point.y, 0.0}, 12.0);
    }
    check(drive.active(), "final hold remains active before tolerance");
    for (int i = 0; i < 100; ++i) {
        drive.Update(0.01, {-100.0, 0.0, 0.0}, 12.0);
    }
    check(drive.active(), "final hold remains active outside tolerance");
    for (int i = 0; i < 19; ++i) {
        drive.Update(0.01, {20.0, 0.0, 0.0}, 12.0);
        check(drive.active(), "final hold requires full settle time");
    }
    drive.Update(0.01, {20.0, 0.0, 0.0}, 12.0);
    check(!drive.active(), "final hold completes after settle time");
}

// Proves both +/-pi branch cuts preserve the nearest angle within 1e-9 rad.
static void test_heading_wrap()
{
    const double pi = std::acos(-1.0);
    check(std::fabs(WrapToNear(pi - 0.01, -pi + 0.01) -
                    (-pi + 0.01) - (-0.02)) < 1e-9,
          "heading wrap across negative branch");
    check(std::fabs(WrapToNear(-pi + 0.01, pi - 0.01) -
                    (pi - 0.01) - 0.02) < 1e-9,
          "heading wrap across positive branch");
}

int main()
{
    test_identification();
    test_safety();
    test_bezier();
    test_scurve();
    test_ladrc();
    test_odometry();
    test_drive_saturation();
    test_curvature_speed_limit();
    test_final_hold();
    test_heading_wrap();
    std::cout << "all control tests passed\n";
    return 0;
}
