#ifndef LADRC2_H
#define LADRC2_H

// Second-order LADRC for y'' = f + b0*u. z1, z2, and z3 estimate position,
// velocity, and total disturbance; all plant quantities use the caller's units.

#include <cmath>

class Ladrc2
{
public:
    // wc is the closed-loop bandwidth in rad/s; wo is the ESO bandwidth in
    // rad/s and defaults to four times wc. b0 is the plant input gain.
    Ladrc2(double wc, double b0, double wo = 0.0);
    void Reset(double position, double velocity = 0.0);
    double Update(double dt, double measured_position, double reference_position,
                  double reference_velocity, double reference_acceleration);
    // Feed the voltage actually delivered to the plant into the next ESO step.
    // This differs from the controller's requested/clamped output when mixing
    // scales both drive sides to respect the available battery voltage.
    void SetAppliedInput(double applied_input);
    double disturbance() const { return z3_; }
    double output() const { return applied_u_; }
    double velocity_estimate() const { return z2_; }
    bool valid() const { return valid_; }
    double input_gain() const { return b0_; }
    double bandwidth() const { return wc_; }

private:
    // applied_u_ is the last physical input used by the ESO. valid_ records
    // whether the latest state and control calculation passed finite checks.
    double wc_, wo_, b0_;
    double z1_, z2_, z3_, applied_u_;
    bool valid_;
};

double IdentifyB0(double steady_acceleration, double step_voltage);
bool FitStepResponse(const double* time, const double* velocity, int count,
                     double& tau, double& steady_velocity);
double ComputeB0(double steady_velocity, double tau, double applied_voltage);

#endif
