#include "odometry.h"
#include <cmath>

// ============================================================================
// Midpoint Arc / Differential-Drive Odometry Implementation
// ============================================================================
// Tracks the robot's (x, y, heading) pose in global coordinates using high-speed
// wheel encoder deltas and the V5 Inertial Sensor rate/rotation.
//
// Features:
// 1. Midpoint angle integration: uses theta + 0.5 * dtheta for second-order
//    arc curvature accuracy rather than Euler approximations.
// 2. Perpendicular tracking wheel offset compensation: subtracts the angular
//    scrub (r * dtheta) induced when rotating about an off-center contact point.
// 3. Finite / NaN sanitization: rejects erroneous sensor readings without
//    corrupting the accumulated field position.

// When true, forward linear displacement is read from the dedicated parallel
// tracking wheel. When false, the average of left/right motor encoders is used.
static const bool kUseParallelWheelForward = false;

Odometry::Odometry(double horizontal_wheel_offset_mm)
    : pose_{0.0, 0.0, 0.0},
      previous_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      heading_bias_(0.0),
      horizontal_offset_mm_(horizontal_wheel_offset_mm),
      velocity_(0.0),
      turn_rate_(0.0),
      initialized_(false)
{
}

void Odometry::Reset(const OdometryMeasurement& m, double bias)
{
    // Verify all sensor inputs are valid numbers before resetting pose
    if (!std::isfinite(m.left_mm) || !std::isfinite(m.right_mm) ||
        !std::isfinite(m.vertical_mm) || !std::isfinite(m.horizontal_mm) ||
        !std::isfinite(m.inertial_heading) || !std::isfinite(m.inertial_rate) ||
        !std::isfinite(bias)) {
        return;
    }

    previous_ = m;
    heading_bias_ = bias;
    pose_ = {0.0, 0.0, m.inertial_heading - bias};
    velocity_ = 0.0;
    turn_rate_ = 0.0;
    initialized_ = true;
}

Pose2 Odometry::Update(double dt, const OdometryMeasurement& m)
{
    // Sanity-check inputs: abort update if any reading contains NaN/Inf
    if (!std::isfinite(m.left_mm) || !std::isfinite(m.right_mm) ||
        !std::isfinite(m.vertical_mm) || !std::isfinite(m.horizontal_mm) ||
        !std::isfinite(m.inertial_heading) || !std::isfinite(m.inertial_rate)) {
        return pose_;
    }

    if (!initialized_) {
        Reset(m, 0.0);
        return pose_;
    }

    if (!(dt > 0.0 && std::isfinite(dt))) {
        return pose_;
    }

    // Compute differential displacements over the elapsed time tick
    const double dl = m.left_mm - previous_.left_mm;
    const double dr = m.right_mm - previous_.right_mm;

    // Linear forward displacement
    const double dv = m.vertical_mm - previous_.vertical_mm;
    const double forward = kUseParallelWheelForward ? dv : 0.5 * (dl + dr);

    // Heading delta in radians directly from inertial gyro rotation
    const double dtheta = m.inertial_heading - previous_.inertial_heading;

    // Lateral displacement: raw perpendicular wheel minus kinematic rotation scrub
    const double dh = m.horizontal_mm - previous_.horizontal_mm;
    const double sideways = dh - (horizontal_offset_mm_ * dtheta);

    // Midpoint heading integration
    const double mid_heading = pose_.heading + (0.5 * dtheta);

    // Transform local robot movements (forward, sideways) into field-centric (x, y)
    pose_.x += forward * std::cos(mid_heading) - sideways * std::sin(mid_heading);
    pose_.y += forward * std::sin(mid_heading) + sideways * std::cos(mid_heading);
    pose_.heading = m.inertial_heading - heading_bias_;

    // Instantaneous rate estimates
    velocity_ = forward / dt;
    turn_rate_ = m.inertial_rate;

    previous_ = m;
    return pose_;
}
