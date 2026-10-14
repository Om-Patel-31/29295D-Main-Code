#ifndef SCURVE_H
#define SCURVE_H

// Jerk-limited seven-phase S-curve in path arc length. The seven phases are
// jerk-up, constant acceleration, jerk-down, cruise, jerk-down, constant
// deceleration, and jerk-up; units are mm, mm/s, mm/s^2, and mm/s^3.

struct ScurveState
{
    // position is path distance in mm; velocity is mm/s; acceleration is
    // mm/s^2; finished identifies the clamped terminal state.
    double position;
    double velocity;
    double acceleration;
    bool finished;
};

class ScurveProfile
{
public:
    // Configure applies vmax, amax, and jmax limits and selects either a
    // cruising or triangular profile. Update advances by dt seconds, while
    // Sample evaluates without changing state.
    ScurveProfile();
    bool Configure(double distance, double vmax, double amax, double jmax);
    ScurveState Update(double dt);
    ScurveState Sample(double time) const;
    double duration() const { return total_time_; }
    double distance() const { return distance_; }
    bool valid() const { return valid_; }

private:
    // tj, ta, and tv are phase durations in s. half_distance and peak_velocity
    // describe one symmetric half of the motion in mm and mm/s.
    double distance_, sign_, vmax_, amax_, jmax_, total_time_;
    double tj_, ta_, tv_, half_distance_, peak_velocity_;
    double elapsed_;
    bool valid_;
    static double HalfDistance(double tj, double ta, double j);
    static double HalfVelocity(double tj, double ta, double j);
    ScurveState SamplePositive(double time) const;
};

bool ValidateScurveLimits(double amax, double jmax, double b0,
                          double wc, double voltage_limit);
// This safety check requires positive b0, acceleration headroom below 70% of
// b0*voltage, and jerk no greater than the requested closed-loop bandwidth.

#endif
