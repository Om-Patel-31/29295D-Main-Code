#include "quintic.h" // Include quintic profile, curve, and PID declarations
#include <algorithm> // Include std::max, std::min
#include <cmath>     // Include math library for fabs and sqrt

// ============================================================================
// 1. Quintic Polynomial Minimum-Jerk Trajectory Profile Implementation
// ============================================================================

// Default constructor initializing profile parameters to zero
QuinticProfile::QuinticProfile()
    : distance_(0.0),                                              // Initialize target displacement to zero
      duration_(0.0),                                              // Initialize execution duration to zero
      elapsed_(0.0)                                                // Initialize elapsed time to zero
{                                                                  // Constructor body
}                                                                  // End constructor

// Configures trajectory duration and constraints from target displacement, max speed, and max acceleration
bool QuinticProfile::Configure(double distance, double vmax, double amax)
{
    if (!std::isfinite(distance) || !std::isfinite(vmax) || !std::isfinite(amax) || // Verify all arguments are finite numbers
        vmax <= 0.0 || amax <= 0.0) {                              // Max velocity and acceleration must be strictly positive
        distance_ = 0.0;                                           // Clear distance
        duration_ = 0.0;                                           // Clear duration
        elapsed_ = 0.0;                                            // Clear elapsed time
        return false;                                              // Reject invalid configuration parameters
    }

    distance_ = distance;                                          // Store total signed displacement
    const double abs_dist = std::fabs(distance);                   // Extract positive displacement magnitude

    if (abs_dist < 1e-6) {                                         // Zero-distance trajectory completes instantly
        duration_ = 0.05;                                          // Set nominal minimum duration
        elapsed_ = 0.0;                                            // Reset elapsed time
        return true;                                               // Return successful configuration
    }

    // Minimum duration required to satisfy peak velocity constraint: v_peak = 1.875 * L / T <= vmax
    const double t_vel = 1.875 * abs_dist / vmax;                  // Time required to stay within velocity bound

    // Minimum duration required to satisfy peak acceleration: a_peak = (10*sqrt(3)/3) * L / T^2 <= amax
    const double t_acc = std::sqrt(5.77350269 * abs_dist / amax);  // Time required to stay within acceleration bound

    duration_ = std::max(t_vel, std::max(t_acc, 0.05));            // Select governing duration with 50ms safety floor
    elapsed_ = 0.0;                                                // Reset elapsed time tracker
    return true;                                                   // Profile successfully configured
}

// Samples reference position, velocity, and acceleration at a specific timestamp
QuinticState QuinticProfile::Sample(double time) const
{
    if (duration_ <= 0.0 || time >= duration_) {                   // Trajectory duration elapsed or invalid
        return {distance_, 0.0, 0.0, true};                        // Return final position with zero velocity and acceleration
    }
    if (time <= 0.0) {                                             // Prior to trajectory start
        return {0.0, 0.0, 0.0, false};                             // Return starting state at rest
    }

    const double tau = time / duration_;                           // Compute normalized time parameter tau in [0.0, 1.0]
    const double tau2 = tau * tau;                                 // Compute tau squared
    const double tau3 = tau2 * tau;                                // Compute tau cubed
    const double tau4 = tau3 * tau;                                // Compute tau to the fourth power
    const double tau5 = tau4 * tau;                                // Compute tau to the fifth power

    // Quintic polynomial position: s(tau) = L * (10*tau^3 - 15*tau^4 + 6*tau^5)
    const double s = distance_ * (10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5);

    // Quintic polynomial velocity: v(tau) = (L / T) * (30*tau^2 - 60*tau^3 + 30*tau^4)
    const double v = (distance_ / duration_) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);

    // Quintic polynomial acceleration: a(tau) = (L / T^2) * (60*tau - 180*tau^2 + 120*tau^3)
    const double a = (distance_ / (duration_ * duration_)) * (60.0 * tau - 180.0 * tau2 + 120.0 * tau3);

    return {s, v, a, false};                                       // Return sampled trajectory reference packet
}

// Advances internal elapsed time by dt and returns the sampled reference state
QuinticState QuinticProfile::Update(double dt)
{
    elapsed_ += dt;                                                // Increment internal elapsed time by timestep
    return Sample(elapsed_);                                       // Return sampled state at updated timestamp
}

// Validates whether peak trajectory acceleration can be commanded within available motor voltage limits
bool ValidateQuinticLimits(double amax, double b0, double wc, double max_voltage)
{
    if (!std::isfinite(amax) || amax <= 0.0 ||                     // Validate maximum acceleration is positive
        !std::isfinite(b0) || b0 <= 0.0 ||                         // Validate plant authority gain is positive
        !std::isfinite(wc) || wc <= 0.0 ||                         // Validate controller bandwidth is positive
        !std::isfinite(max_voltage) || max_voltage <= 0.0) {       // Validate maximum voltage is positive
        return false;                                              // Reject invalid parameters
    }
    // Acceleration headroom: feedforward voltage requirement (amax / b0) must not exceed 85% of battery voltage
    const double u_ff = amax / b0;                                 // Feedforward voltage demanded by peak acceleration
    return (u_ff <= 0.85 * max_voltage);                           // Ensure at least 15% voltage headroom remains for feedback
}

// ============================================================================
// 2. Quintic Polynomial Joystick Curve Shaper Implementation
// ============================================================================

// Constructor for QuinticCurve shaper: sets deadband and polynomial blending weights
QuinticCurve::QuinticCurve(double deadband_pct, double linear_weight, double quintic_weight)
    : deadband_(deadband_pct),                                     // Set deadband threshold
      linear_weight_(linear_weight),                               // Set linear term weighting
      quintic_weight_(quintic_weight)                              // Set quintic term weighting
{                                                                  // Constructor body
}                                                                  // End constructor

// Evaluates quintic polynomial curve on a raw joystick input [-100, 100]
double QuinticCurve::Calculate(double input_pct) const
{
    if (std::fabs(input_pct) < deadband_) {                        // Check if joystick is inside deadband
        return 0.0;                                                // Return zero neutral response
    }
    const double sign = (input_pct > 0.0) ? 1.0 : -1.0;            // Preserve input sign
    const double max_range = 100.0 - deadband_;                    // Calculate active throw range beyond deadband
    const double norm = (std::fabs(input_pct) - deadband_) / max_range; // Normalize deflection into [0.0, 1.0]
    const double norm5 = norm * norm * norm * norm * norm;         // Calculate 5th power quintic response
    const double shaped = (linear_weight_ * norm) + (quintic_weight_ * norm5); // Blend linear and quintic terms
    return sign * shaped * 100.0;                                  // Rescale shaped deflection back to percentage
}

// ============================================================================
// 3. Discrete PID Controller Implementation
// ============================================================================

// Constructor: initializes gains, anti-windup clamp, and resets internal state
PID::PID(double kp, double ki, double kd, double integral_max)
    : kp_(kp),                                                     // Set proportional gain
      ki_(ki),                                                     // Set integral gain
      kd_(kd),                                                     // Set derivative gain
      integral_max_(integral_max),                                 // Set maximum integral clamp
      integral_(0.0),                                              // Clear integral accumulator
      prev_error_(0.0),                                            // Clear previous error
      hold_timer_(0.0),                                            // Clear settle hold timer
      first_run_(true)                                             // Set first-run flag to suppress initial derivative spike
{                                                                  // Constructor body
}                                                                  // End of constructor

// Updates PID gains and anti-windup limit dynamically
void PID::SetGains(double kp, double ki, double kd, double integral_max)
{
    kp_ = kp;                                                      // Update proportional gain
    ki_ = ki;                                                      // Update integral gain
    kd_ = kd;                                                      // Update derivative gain
    integral_max_ = integral_max;                                  // Update integral clamp
}

// Resets internal accumulator and derivative history
void PID::Reset()
{
    integral_ = 0.0;                                               // Clear accumulated integral
    prev_error_ = 0.0;                                             // Clear previous cycle error
    hold_timer_ = 0.0;                                             // Clear settle timer
    first_run_ = true;                                             // Reset first run flag
}

// Calculates PID control output given an error value and timestep dt
double PID::Calculate(double error, double dt)
{
    if (!std::isfinite(error) || !std::isfinite(dt) || dt <= 0.0) { // Verify inputs are valid finite numbers
        return 0.0;                                                // Return zero on invalid input
    }

    const double p_term = kp_ * error;                             // Calculate proportional term (P = kp * error)

    integral_ += error * dt;                                       // Integrate error over time (Euler integration)
    if (integral_max_ > 0.0) {                                     // Check if anti-windup clamping is enabled
        if (integral_ > integral_max_) {                           // Integral exceeds positive clamp
            integral_ = integral_max_;                             // Clamp to positive maximum
        } else if (integral_ < -integral_max_) {                   // Integral exceeds negative clamp
            integral_ = -integral_max_;                            // Clamp to negative maximum
        }
    }
    const double i_term = ki_ * integral_;                         // Calculate integral term (I = ki * integral)

    double derivative = 0.0;                                       // Initialize derivative value
    if (!first_run_) {                                             // Compute derivative only after first cycle
        derivative = (error - prev_error_) / dt;                   // Calculate error rate of change (dE / dt)
    } else {                                                       // First iteration
        first_run_ = false;                                        // Clear first run flag
    }
    const double d_term = kd_ * derivative;                        // Calculate derivative term (D = kd * derivative)

    prev_error_ = error;                                           // Store current error for next cycle's derivative

    return p_term + i_term + d_term;                               // Return total control signal (P + I + D)
}

// Calculates PID control output given reference target and measured feedback
double PID::Calculate(double target, double measured, double dt)
{
    return Calculate(target - measured, dt);                       // Forward call using error = target - measured
}

// Evaluates whether error has stayed within tolerance for the specified duration
bool PID::IsSettled(double tolerance, double dt, double hold_time_s)
{
    if (std::fabs(prev_error_) <= tolerance) {                     // Current error is within allowable tolerance
        hold_timer_ += dt;                                         // Accumulate settled hold time
    } else {                                                       // Error exceeded tolerance
        hold_timer_ = 0.0;                                         // Reset hold timer
    }
    return hold_timer_ >= hold_time_s;                             // Return true if error remained within bounds long enough
}
