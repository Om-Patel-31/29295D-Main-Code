#ifndef LADRC2_H
#define LADRC2_H

#include <cmath> // Include math library for fabs and isfinite checks

// ============================================================================
// Second-Order Linear Active Disturbance Rejection Control (LADRC)
// ============================================================================
// Models the plant as y'' = f + b0*u, where f represents total lumped disturbances
// (external friction, slope, collisions, modeling uncertainties) and b0 is the control gain.
// A 3rd-order Extended State Observer (ESO) estimates:
//   z1: output position (same units as measured y)
//   z2: output velocity (units/s)
//   z3: total disturbance f (units/s^2)
//
// Disturbance handling features:
//   - max_disturb_: Clamps z3 to prevent observer windup / peaking during motor saturation.
//   - applied_u_: Tracks the actual physical voltage applied to actuators (anti-windup).
// ============================================================================
class Ladrc2
{
public:
    // Constructor initializes bandwidths, input gain, disturbance limit, and resets state
    Ladrc2(double wc, double b0, double wo = 0.0, double max_disturb = 0.0);

    // Resets observer state variables to specified position and velocity
    void Reset(double position, double velocity = 0.0);

    // Runs one discrete ESO estimation step and calculates PD feedback control signal
    double Update(double dt, double measured_position, double reference_position,
                  double reference_velocity, double reference_acceleration);

    // Feeds back the actual actuator voltage applied after common-mode mixer clamping
    void SetAppliedInput(double applied_input);

    // Sets or updates the maximum disturbance acceleration limit clamp
    void SetMaxDisturbance(double max_disturb) { max_disturb_ = max_disturb; }

    // Read current disturbance estimate z3 (units/s^2)
    double disturbance() const { return z3_; }

    // Read current commanded/applied control effort
    double output() const { return applied_u_; }

    // Read current velocity estimate z2 (units/s)
    double velocity_estimate() const { return z2_; }

    // Check if the observer state and calculations are numerically valid (no NaN/Inf)
    bool valid() const { return valid_; }

    // Read the control authority gain b0
    double input_gain() const { return b0_; }

    // Read the controller bandwidth wc (rad/s)
    double bandwidth() const { return wc_; }

    // Read the observer bandwidth wo (rad/s)
    double observer_bandwidth() const { return wo_; }

    // Read the maximum disturbance clamp
    double max_disturbance() const { return max_disturb_; }

private:
    double wc_;          // Controller closed-loop bandwidth (rad/s)
    double wo_;          // Observer closed-loop bandwidth (rad/s)
    double b0_;          // Plant input authority gain ((units/s^2)/Volt)
    double max_disturb_; // Maximum disturbance limit clamp to prevent observer windup
    double z1_;          // Observer state 1: estimated position
    double z2_;          // Observer state 2: estimated velocity
    double z3_;          // Observer state 3: estimated lumped disturbance f
    double applied_u_;   // Actual voltage delivered to actuator in previous step
    bool valid_;         // Flag indicating whether all internal states are finite and valid
};

#endif // LADRC2_H
