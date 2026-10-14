#ifndef DRIVE_H
#define DRIVE_H

// Coordinates a longitudinal and heading LADRC pair along a Bezier path.
// The mixer uses V_L=U_long-U_turn and V_R=U_long+U_turn, with voltages in V.

#include "bezier.h"
#include "ladrc2.h"
#include "odometry.h"
#include "scurve.h"

struct DriveVoltages
{
    // Motor commands after common-mode scaling, in V.
    double left;
    double right;
};

double WrapToNear(double angle, double reference);

class DriveController
{
public:
    // wc values are rad/s and b0 values are plant gains. StartPath receives
    // the measured path-frame pose so a restart does not assume the origin.
    DriveController(double longitudinal_wc, double longitudinal_b0,
                    double heading_wc, double heading_b0);
    bool StartPath(const BezierPath& path, const Pose2& measured,
                   double vmax, double amax, double jmax);
    // Read the supply voltage on the VEX side and pass it into the controller.
    DriveVoltages Update(double dt, const Pose2& measured, double supply_voltage);
    void Reset(const Pose2& measured);
    bool active() const { return active_; }
    bool faulted() const { return faulted_; }
    double elapsed() const { return elapsed_; }
    double longitudinal_velocity_estimate() const {
        return longitudinal_.velocity_estimate();
    }
    const ScurveProfile& profile() const { return profile_; }

private:
    // elapsed_ and hold_time_ are seconds. active_ tracks path execution;
    // faulted_ distinguishes invalid LADRC data from ordinary completion.
    BezierPath path_;
    ScurveProfile profile_;
    Ladrc2 longitudinal_, heading_;
    double elapsed_;
    double hold_time_;
    bool active_;
    bool faulted_;
};

#endif
