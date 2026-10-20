#include "ladrc2.h"  // Include LADRC class declaration
#include "tuning.h"  // Include global tuning configuration for voltage limits
#include <cmath>     // Include math functions for isfinite and fabs

// ============================================================================
// Second-Order LADRC & Extended State Observer Implementation
// ============================================================================

// Constructor: initializes controller/observer gains, disturbance clamp, and state variables
Ladrc2::Ladrc2(double wc, double b0, double wo, double max_disturb)
    : wc_(wc),                                        // Set controller bandwidth in rad/s
      wo_(wo > 0.0 ? wo : 4.0 * wc),                  // Default observer bandwidth to 4x controller bandwidth
      b0_(b0),                                        // Set plant input gain
      max_disturb_(max_disturb),                      // Set maximum disturbance acceleration limit clamp
      z1_(0.0),                                       // Initialize position estimate to zero
      z2_(0.0),                                       // Initialize velocity estimate to zero
      z3_(0.0),                                       // Initialize disturbance estimate to zero
      applied_u_(0.0),                                // Initialize applied actuator voltage to zero
      valid_(false)                                   // Mark state as uninitialized until Reset() or Update()
{                                                     // Constructor body
}                                                     // End of constructor

// Resets observer states to known initial position and velocity
void Ladrc2::Reset(double position, double velocity)
{
    z1_ = position;                                   // Set position estimate to measured start position
    z2_ = velocity;                                   // Set velocity estimate to start velocity
    z3_ = 0.0;                                        // Clear disturbance estimate
    applied_u_ = 0.0;                                 // Clear previous applied control effort
    valid_ = true;                                    // Mark observer state as valid and initialized
}

// Feeds back the actual voltage applied to the plant after system voltage clamping
void Ladrc2::SetAppliedInput(double applied_input)
{
    if (std::isfinite(applied_input)) {               // Verify input is a valid finite number
        applied_u_ = applied_input;                   // Record actual delivered voltage for next ESO step
    } else {                                          // Actuator command is NaN or infinite
        applied_u_ = 0.0;                             // Default applied effort to safe zero
        valid_ = false;                               // Invalidate controller state
    }
}

// Runs one discrete-time ESO estimation update and computes PD feedback control law
double Ladrc2::Update(double dt, double y, double r1, double r2, double r3)
{
    // Validate timestep, sensor measurement, references, and gains
    if (!(dt > 0.0 && dt < 0.25 && std::isfinite(dt) && // Timestep must be positive and under 250ms
          std::isfinite(y) &&                         // Measured output must be a finite number
          std::isfinite(r1) &&                        // Reference position must be a finite number
          std::isfinite(r2) &&                        // Reference velocity must be a finite number
          std::isfinite(r3) &&                        // Reference acceleration must be a finite number
          std::isfinite(b0_) && std::fabs(b0_) > 1e-9 && // Input gain must be finite and non-zero
          std::isfinite(wc_) &&                       // Controller bandwidth must be a finite number
          std::isfinite(wo_))) {                      // Observer bandwidth must be a finite number
        valid_ = false;                               // Invalidate controller on invalid inputs
        applied_u_ = 0.0;                             // Clear applied effort
        return 0.0;                                   // Return safe zero command
    }

    // Compute observer estimation error: difference between estimated and measured position
    const double e = z1_ - y;                         // Output estimation error (e = z1 - y)

    // Compute discrete ESO observer gain coefficients based on observer bandwidth wo
    const double b1 = 3.0 * wo_;                      // Observer correction gain for position estimate
    const double b2 = 3.0 * wo_ * wo_;                // Observer correction gain for velocity estimate
    const double b3 = wo_ * wo_ * wo_;                // Observer correction gain for disturbance estimate

    // Snapshot previous observer states prior to Euler integration
    const double old_z1 = z1_;                        // Prior position estimate
    const double old_z2 = z2_;                        // Prior velocity estimate
    const double old_z3 = z3_;                        // Prior disturbance estimate

    // Update state estimates using forward Euler integration
    z1_ = old_z1 + dt * (old_z2 - b1 * e);            // Propagate position estimate
    z2_ = old_z2 + dt * (old_z3 + b0_ * applied_u_ - b2 * e); // Propagate velocity with plant dynamics
    z3_ = old_z3 + dt * (-b3 * e);                    // Propagate lumped disturbance estimate

    // Anti-windup disturbance clamping: prevent observer runaway during stalls or collisions
    if (max_disturb_ > 0.0) {                         // Check if disturbance clamping is active
        if (z3_ > max_disturb_) {                     // Disturbance exceeds positive acceleration limit
            z3_ = max_disturb_;                       // Clamp disturbance to positive threshold
        } else if (z3_ < -max_disturb_) {             // Disturbance exceeds negative acceleration limit
            z3_ = -max_disturb_;                      // Clamp disturbance to negative threshold
        }
    }

    // Compute state-feedback control law: u0 = kp*(r1 - z1) + kd*(r2 - z2) + (r3 - z3)
    const double kp = wc_ * wc_;                      // Proportional gain (kp = wc^2)
    const double kd = 2.0 * wc_;                      // Derivative gain (kd = 2*wc)
    const double u0 = kp * (r1 - z1_) + kd * (r2 - z2_) + r3 - z3_; // Disturbance-canceling control law
    const double raw = u0 / b0_;                      // Scale by control authority to obtain voltage

    // Verify all computed values remain strictly finite
    if (!std::isfinite(raw) || !std::isfinite(z1_) || // Check raw voltage and position estimate
        !std::isfinite(z2_) || !std::isfinite(z3_)) { // Check velocity and disturbance estimates
        valid_ = false;                               // Invalidate controller if arithmetic exploded
        applied_u_ = 0.0;                             // Clear applied command
        return 0.0;                                   // Return zero voltage
    }

    // Clamp output voltage to maximum allowable system voltage
    const double v_max = tuning.max_voltage;          // System voltage upper bound (12.0 V)
    if (raw > v_max) {                                // Upper voltage limit exceeded
        applied_u_ = v_max;                           // Saturate at maximum positive voltage
    } else if (raw < -v_max) {                        // Lower voltage limit exceeded
        applied_u_ = -v_max;                          // Saturate at maximum negative voltage
    } else {                                          // Command is within safe battery limits
        applied_u_ = raw;                             // Apply unsaturated control command
    }

    valid_ = true;                                    // Mark control calculation as valid
    return applied_u_;                                // Return final motor command in Volts
}
