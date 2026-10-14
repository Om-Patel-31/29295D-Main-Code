#include "ladrc2.h" // Pulls in the class declaration and its member definitions so this source can implement the LADRC methods.
#include "config.h"

// This implementation uses a discrete Euler observer. The ESO reconstructs
// the unmeasured disturbance f so the control law can reject it online.
// ------------------------------------------------------------
// Constructor for the LADRC controller. It initializes the controller
// gains and internal state, and ensures the observer bandwidth is never
// allowed to collapse to zero or negative values.
// ------------------------------------------------------------
Ladrc2::Ladrc2(double wc, double b0, double wo)
    : wc_(wc), wo_(wo > 0.0 ? wo : 4.0 * wc), b0_(b0), // Stores the closed-loop bandwidth, observer bandwidth, and input gain.
      z1_(0.0), z2_(0.0), z3_(0.0), applied_u_(0.0), valid_(false) {} // Resets internal state variables and marks the controller as not yet initialized.

// ----------------------------------------------------------------------------
// Resets the internal observer state to a known operating point before a new
// control sequence begins. This is useful when starting a fresh estimation run.
// ----------------------------------------------------------------------------
void Ladrc2::Reset(double position, double velocity)
{
    z1_ = position; z2_ = velocity; z3_ = 0.0; applied_u_ = 0.0; valid_ = true; // Stores the initial position and velocity, clears the higher-order state, and marks the estimate as valid.
}

void Ladrc2::SetAppliedInput(double applied_input)
{
    // The ESO must model what the motor received, not a request that may have
    // been reduced by drive-side common-mode saturation.
    if (std::isfinite(applied_input)) {
        applied_u_ = applied_input;
    } else {
        applied_u_ = 0.0;
        valid_ = false;
    }
}

// --------------------------------------------------------------------------------
// Runs one control-update step for the LADRC observer/controller. It validates the
// input data, propagates the internal state, computes a raw control signal, and
// clamps it to safeguard stability.
// --------------------------------------------------------------------------------
double Ladrc2::Update(double dt, double y, double r1, double r2, double r3)
{
    // r1, r2, and r3 are desired position, velocity, and acceleration. The
    // feedforward r3 belongs in u0 because it is a reference term, not plant
    // input and therefore must not be injected into the ESO state.
    if (!(dt > 0.0 && dt < 0.25 && std::isfinite(dt) && // Requires a positive, reasonably small timestep and finite values for all essential inputs.
          std::isfinite(y) && std::isfinite(r1) && std::isfinite(r2) && // Checks the measured output and reference states are all finite numbers.
          std::isfinite(r3) && std::isfinite(b0_) && std::fabs(b0_) > 1e-9 && // Confirms the input gain is finite and nonzero so division is safe.
          std::isfinite(wc_) && std::isfinite(wo_))) { // Ensures the controller bandwidth settings are finite before continuing.
        valid_ = false; applied_u_ = 0.0; return 0.0; // Invalid input data disables the controller and produces a safe zero output.
    }
    // z1, z2, and z3 estimate position, velocity, and total disturbance under
    // the model y'' = f + b0*u; e drives the observer correction terms.
    const double e = z1_ - y; // Computes the tracking error between the estimated state and the measured output.
    const double b1 = 3.0 * wo_; // First observer gain for the tracking-error correction term.
    const double b2 = 3.0 * wo_ * wo_; // Second observer gain that scales with the observer bandwidth squared.
    const double b3 = wo_ * wo_ * wo_; // Third observer gain that scales with the observer bandwidth cubed.
    const double old_z1 = z1_, old_z2 = z2_, old_z3 = z3_; // Saves the previous observer states before updating them with the new dynamics.
    z1_ = old_z1 + dt * (old_z2 - b1 * e); // Updates the first observer state using a damped error correction term.
    z2_ = old_z2 + dt * (old_z3 + b0_ * applied_u_ - b2 * e); // Updates the second observer state using the dynamics term plus the current control effort.
    z3_ = old_z3 + dt * (-b3 * e); // Updates the third observer state to reflect the higher-order error dynamics.
    // u0 is the state-feedback law. wc sets tracking response speed; wo sets
    // how quickly the ESO reconstructs disturbance and unmeasured velocity.
    const double raw = (wc_ * wc_ * (r1 - z1_) + // Forms the state-feedback term for the first reference difference.
                        2.0 * wc_ * (r2 - z2_) + r3 - z3_) / b0_; // Adds the second and third reference differences and scales by the input gain.
    if (!std::isfinite(raw) || !std::isfinite(z1_) || // Confirms the computed control law and state estimates remain numerically valid.
        !std::isfinite(z2_) || !std::isfinite(z3_)) {
        valid_ = false; applied_u_ = 0.0; return 0.0; // If the computation becomes non-finite, the controller is invalidated and a safe zero command is returned.
    }
    applied_u_ = raw > config::max_voltage ? config::max_voltage :
                 (raw < -config::max_voltage ? -config::max_voltage : raw); // Limits the output to the configured voltage.
    valid_ = true;
    return applied_u_; // Returns the clamped control action for the current time step.
}

// ------------------------------------------------------------------------------
// Placeholder function for identifying the input gain b0 from a steady-state
// acceleration and a step-voltage estimate. It is intentionally left unimplemented
// in this file and simply returns zero until a real estimator is supplied.
// ------------------------------------------------------------------------------
double IdentifyB0(double steady_acceleration, double step_voltage)
{
    (void)steady_acceleration; // Suppresses the unused-parameter warning because the current version does not use this value.
    (void)step_voltage; // Suppresses the unused-parameter warning because this estimator does not yet depend on the step voltage.
    return 0.0; // Returns zero as a placeholder indicator that no valid estimate is available.
}

// -------------------------------------------------------------------------------
// Fits a first-order step response model to the measured time/velocity data by
// scanning a range of candidate time constants and selecting the one that minimizes
// the squared error between the model and the observed response.
// -------------------------------------------------------------------------------
bool FitStepResponse(const double* time, const double* velocity, int count,
                     double& tau, double& steady_velocity)
{
    if (!time || !velocity || count < 3) return false; // Rejects invalid pointers or too-few samples before fitting the model.
    double best_error = 1e300; // Tracks the smallest residual error seen so far during the search.
    double best_tau = 0.0; // Stores the candidate time constant with the smallest error.
    double best_v = 0.0; // Stores the steady-state velocity associated with the best candidate.
    for (int i = 0; i < 4001; ++i) { // Searches a broad logarithmic range of time constants to find the best fit.
        const double candidate = 0.01 * std::pow(10.0, 3.0 * i / 4000.0); // Uses a logarithmic sweep from 0.01 to 1000 seconds to cover a wide model range.
        double aa = 0.0, ab = 0.0; // Initializes the numerator and denominator terms for the least-squares fit.
        for (int n = 0; n < count; ++n) { // Evaluates the fit contribution for every measured sample in the dataset.
            if (!std::isfinite(time[n]) || !std::isfinite(velocity[n]) || // Verifies each data point is finite and physically meaningful before use.
                time[n] < 0.0) return false; // Returns failure when the input data contain invalid or negative timestamps.
            const double basis = 1.0 - std::exp(-time[n] / candidate); // Computes the first-order response basis function for this time constant.
            aa += basis * basis; // Accumulates the squared basis term needed for the least-squares denominator.
            ab += basis * velocity[n]; // Accumulates the velocity-weighted basis term needed for the least-squares numerator.
        }
        if (aa <= 0.0) continue; // Skips degenerate fits where the basis function does not contribute any information.
        const double v = ab / aa; // Solves for the best steady-state level for the current time-constant candidate.
        double error = 0.0; // Tracks the total squared residual error for this candidate model.
        for (int n = 0; n < count; ++n) { // Computes the error across all samples for this candidate fit.
            const double residual = velocity[n] - v * (1.0 - std::exp(-time[n] / candidate)); // Measures the difference between the observed velocity and the model prediction.
            error += residual * residual; // Sums the squared residuals to represent the model quality.
        }
        if (error < best_error) { // Keeps the candidate only if it improves on the previous best match.
            best_error = error; // Saves the lower error value as the new benchmark.
            best_tau = candidate; // Stores the improved time constant candidate.
            best_v = v; // Stores the corresponding steady-state velocity estimate.
        }
    }
    if (!(best_tau > 0.0) || !std::isfinite(best_v)) return false; // Verifies that a valid positive time constant and finite steady-state value were found.
    tau = best_tau; // Assigns the best-fit time constant to the output parameter.
    steady_velocity = best_v; // Assigns the best-fit steady-state velocity to the output parameter.
    return true; // Indicates that a valid model fit was successfully produced.
}

// ------------------------------------------------------------------------------
// Computes the lumped input-gain term b0 from a steady-state velocity, the time
// constant tau, and the applied actuator voltage. This is the inverse relation
// implied by the first-order steady-state model.
// ------------------------------------------------------------------------------
double ComputeB0(double steady_velocity, double tau, double applied_voltage)
{
    if (!std::isfinite(steady_velocity) || !std::isfinite(tau) || // Validates the steady-state velocity and time constant are finite numbers.
        !std::isfinite(applied_voltage) || tau <= 0.0 || // Ensures the applied voltage is finite and the time constant is strictly positive.
        std::fabs(applied_voltage) < 1e-9) return 0.0; // Rejects invalid or effectively zero voltage to avoid division by zero.
    return steady_velocity / (tau * applied_voltage); // Solves for the effective input gain using the steady-state first-order relation.
}
