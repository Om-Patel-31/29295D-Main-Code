#ifndef MOTION_H
#define MOTION_H

// ============================================================================
// Autonomous Motion Engine & Drivetrain LADRC Path Tracking Controller
// ============================================================================
// Coordinates longitudinal distance and heading tracking along Bezier paths
// using 2nd-order LADRC disturbance rejection with quintic trajectory profiling.
// Also provides the blocking autonomous motion API:
//   - drive(distance_in, [speed_pct], [timeout_s])
//   - turn(angle_deg, [speed_pct], [timeout_s])
//   - turn_to(heading_deg, [speed_pct], [timeout_s])
//   - bdrive(forward_in, right_in, end_heading_deg, [speed_pct], [timeout_s])
//   - sdrive()
// ============================================================================

#include "bezier.h"   // Include Bezier curve geometry and parameterization
#include "ladrc2.h"   // Include LADRC and Extended State Observer
#include "odometry.h" // Include Pose2 and Odometry structures
#include "quintic.h"  // Include QuinticProfile trajectory generator

// Structure holding left and right drivetrain motor voltages in Volts
struct DriveVoltages
{
    double left;  // Motor voltage delivered to left drive train (Volts)
    double right; // Motor voltage delivered to right drive train (Volts)
};

// Angles wrapping helper: selects angle representation nearest reference to avoid +/-180 wrap jumps
double WrapToNear(double angle, double reference);

// Main drivetrain path-following controller class
class DriveController
{
public:
    // Default constructor: initializes longitudinal and heading controllers using global tuning
    DriveController();

    // Custom constructor: initializes longitudinal and heading controllers with explicit gains
    DriveController(double longitudinal_wc, double longitudinal_b0,
                    double heading_wc, double heading_b0);

    // Configures quintic trajectory along a Bezier path and resets controllers
    bool StartPath(const BezierPath& path, const Pose2& measured,
                   double vmax, double amax);

    // Executes one discrete control cycle, computing left and right motor voltages
    DriveVoltages Update(double dt, const Pose2& measured, double supply_voltage);

    // Resets controller and observer states to idle
    void Reset(const Pose2& measured);

    // Sets optional linear and rotational velocity settling thresholds
    void SetSettleSpeeds(double speed, double turn_rate);

    // Returns true if path execution is currently active
    bool active() const { return active_; }

    // Returns true if a controller calculation error or numerical fault occurred
    bool faulted() const { return faulted_; }

    // Returns elapsed time since start of path execution in seconds
    double elapsed() const { return elapsed_; }

    // Returns the current longitudinal velocity estimate from the ESO in mm/s
    double longitudinal_velocity_estimate() const {
        return longitudinal_.velocity_estimate();
    }

    // Returns the longitudinal ESO disturbance estimate in mm/s^2.
    double longitudinal_disturbance_estimate() const {
        return longitudinal_.disturbance();
    }

    // Returns current longitudinal commanded voltage in Volts.
    double longitudinal_output() const {
        return longitudinal_.output();
    }

    // Returns the current rotational velocity estimate from the ESO in rad/s
    double heading_velocity_estimate() const {
        return heading_.velocity_estimate();
    }

    // Provides read access to the underlying quintic motion profile
    const QuinticProfile& profile() const { return profile_; }

private:
    BezierPath path_;                 // Planned Bezier path geometry
    QuinticProfile profile_;          // C^2 continuous quintic minimum-jerk motion profile
    Ladrc2 longitudinal_;             // Linear displacement LADRC controller
    Ladrc2 heading_;                  // Angular heading LADRC controller
    double elapsed_;                  // Total elapsed active trajectory time (seconds)
    double hold_time_;                // Accumulated duration within settle tolerances (seconds)
    double settle_speed_;             // Maximum linear speed threshold to consider settled (mm/s)
    double settle_turn_rate_;         // Maximum angular rate threshold to consider settled (rad/s)
    bool active_;                     // Flag indicating if path execution is in progress
    bool faulted_;                    // Flag indicating if controller encountered numerical failure
};

// ============================================================================
// Blocking Autonomous Motion API (Inches & Degrees)
// ============================================================================

// Executes a straight drive: distance in inches, optional speed percentage (default 100%), optional timeout
bool drive(double distance_in, double speed_pct = 100.0, double timeout_s = 0.0);

// Executes a turn in place by a relative angle in degrees (+90 = turn right 90 deg, -90 = turn left 90 deg)
bool turn(double angle_deg, double speed_pct = 100.0, double timeout_s = 0.0);

// Executes a turn in place to an absolute heading in degrees relative to the field 0-degree reference
bool turn_to(double heading_deg, double speed_pct = 100.0, double timeout_s = 0.0);

// Executes a curved Bezier drive: forward displacement in inches, sideways in inches, end heading in degrees
bool bezier_drive(double forward_in, double right_in, double end_heading_deg,
                  double speed_pct = 100.0, double timeout_s = 0.0);

// Short convenience alias for bezier_drive: bdrive(forward_in, sideways_in, end_heading_deg, [speed_pct], [timeout_s])
inline bool bdrive(double forward_in, double right_in, double end_heading_deg,
                   double speed_pct = 100.0, double timeout_s = 0.0)
{
    return bezier_drive(forward_in, right_in, end_heading_deg, speed_pct, timeout_s); // Forward call to bezier_drive
}

// Actively stops and holds all 4 drivetrain motors using motor brake mode
void sdrive();

#endif // MOTION_H
