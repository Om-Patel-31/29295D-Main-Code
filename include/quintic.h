#ifndef QUINTIC_H
#define QUINTIC_H

#include <cmath>     // Include math library for fabs and sqrt

// ============================================================================
// Quintic Polynomial Trajectory, Driver Input Shaper, and PID System
// ============================================================================
// Unifies all smooth motion profiling and feedback mechanisms:
// 1. QuinticProfile: C^2 continuous minimum-jerk profile for autonomous motion.
// 2. QuinticCurve: 5th-order polynomial joystick shaper (used by Drivetrain).
// 3. PID: Discrete feedback controller with anti-windup clamping.
// ============================================================================

// State packet produced by sampling a quintic trajectory profile
struct QuinticState
{
    double position;     // Reference position r1 (displacement in mm or angle in rad)
    double velocity;     // Reference velocity r2 (speed in mm/s or turn rate in rad/s)
    double acceleration; // Reference acceleration feedforward r3 (in mm/s^2 or rad/s^2)
    bool done;           // Flag indicating whether trajectory duration has completed
};

// 5th-order (quintic) polynomial minimum-jerk trajectory profile generator
class QuinticProfile
{
public:
    // Default constructor initializing internal state to zero
    QuinticProfile();

    // Configures trajectory duration and constraints from distance, max speed, and max acceleration
    bool Configure(double distance, double vmax, double amax);

    // Evaluates reference position, velocity, and acceleration at elapsed time t
    QuinticState Sample(double time) const;

    // Advances internal elapsed time by dt and returns the sampled state
    QuinticState Update(double dt);

    // Returns total planned duration in seconds
    double duration() const { return duration_; }

    // Returns target displacement distance
    double distance() const { return distance_; }

private:
    double distance_; // Total path displacement distance
    double duration_; // Total profile execution duration (seconds)
    double elapsed_;  // Accumulated execution time (seconds)
};

// Helper validating whether profile acceleration limits remain within motor voltage capabilities
bool ValidateQuinticLimits(double amax, double b0, double wc, double max_voltage);

// Single-axis quintic polynomial curve shaper for manual joystick control
class QuinticCurve
{
public:
    // Constructor setting deadband percentage and linear/quintic polynomial weights
    QuinticCurve(double deadband_pct = 5.0, double linear_weight = 0.25, double quintic_weight = 0.75);

    // Evaluates shaped percentage from raw joystick deflection in [-100, 100]
    double Calculate(double input_pct) const;

    // Sets deadband threshold percentage
    void SetDeadband(double deadband_pct) { deadband_ = deadband_pct; }

    // Read deadband threshold percentage
    double deadband() const { return deadband_; }

private:
    double deadband_;       // Deadband threshold percentage
    double linear_weight_;  // Linear component weighting coefficient
    double quintic_weight_; // Quintic (5th power) component weighting coefficient
};

// Discrete PID controller with anti-windup clamping and settle detection
class PID
{
public:
    // Constructor initializes PID gains and anti-windup clamp
    PID(double kp = 0.0, double ki = 0.0, double kd = 0.0, double integral_max = 0.0);

    // Updates controller gains online
    void SetGains(double kp, double ki, double kd, double integral_max = 0.0);

    // Computes control output given current tracking error and loop timestep dt
    double Calculate(double error, double dt);

    // Computes control output given reference target and measured feedback
    double Calculate(double target, double measured, double dt);

    // Clears accumulated integral sum and resets derivative history
    void Reset();

    // Checks whether the error has settled within tolerance for the specified duration
    bool IsSettled(double tolerance, double dt, double hold_time_s = 0.1);

    // Read proportional gain
    double kp() const { return kp_; }

    // Read integral gain
    double ki() const { return ki_; }

    // Read derivative gain
    double kd() const { return kd_; }

    // Read current error
    double error() const { return prev_error_; }

    // Read current integral accumulator
    double integral() const { return integral_; }

private:
    double kp_;           // Proportional gain coefficient
    double ki_;           // Integral gain coefficient
    double kd_;           // Derivative gain coefficient
    double integral_max_; // Anti-windup maximum integral limit clamp
    double integral_;     // Accumulated discrete integral sum
    double prev_error_;   // Tracking error from previous update cycle
    double hold_timer_;   // Accumulated time tracking error has remained within tolerance
    bool first_run_;      // Initialization flag to avoid derivative kick on first update
};

#endif // QUINTIC_H
