#include "scurve.h"
#include <algorithm>
#include <cassert>
#include <cmath>

// The profile is symmetric in magnitude. A negative distance is handled by
// applying sign_ to position, velocity, and acceleration after sampling.
ScurveProfile::ScurveProfile()
    : distance_(0), sign_(1), vmax_(0), amax_(0), jmax_(0), total_time_(0),
      tj_(0), ta_(0), tv_(0), half_distance_(0), peak_velocity_(0), elapsed_(0), valid_(false) {}

double ScurveProfile::HalfVelocity(double tj, double ta, double j)
{ return j * tj * (tj + ta); }

double ScurveProfile::HalfDistance(double tj, double ta, double j)
{
    const double a = j * tj;
    const double v1 = 0.5 * j * tj * tj;
    const double x1 = j * tj * tj * tj / 6.0;
    const double x2 = v1 * ta + 0.5 * a * ta * ta;
    const double v2 = v1 + a * ta;
    const double x3 = v2 * tj + 0.5 * a * tj * tj - j * tj * tj * tj / 6.0;
    return x1 + x2 + x3;
}

bool ScurveProfile::Configure(double distance, double vmax, double amax, double jmax)
{
    // Long moves use all seven phases; short moves bisect the peak velocity
    // until the jerk and acceleration limits fit without a cruise phase.
    valid_ = false; elapsed_ = 0.0;
    if (!std::isfinite(distance) || !std::isfinite(vmax) ||
        !std::isfinite(amax) || !std::isfinite(jmax) ||
        vmax <= 0 || amax <= 0 || jmax <= 0) return false;
    distance_ = std::fabs(distance); sign_ = distance < 0 ? -1.0 : 1.0;
    vmax_ = vmax; amax_ = amax; jmax_ = jmax;
    const double tj_nom = amax / jmax;
    const double ta_nom = std::max(0.0, vmax / amax - tj_nom);
    const double d_nom = HalfDistance(tj_nom, ta_nom, jmax);
    if (distance_ >= 2.0 * d_nom) {
        tj_ = tj_nom; ta_ = ta_nom; tv_ = (distance_ - 2.0 * d_nom) /
            HalfVelocity(tj_, ta_, jmax);
    } else {
        double lo = 0.0, hi = vmax;
        for (int i = 0; i < 80; ++i) {
            const double mid = (lo + hi) * 0.5;
            const double tj = std::min(amax / jmax, std::sqrt(mid / jmax));
            const double ta = std::max(0.0, mid / (jmax * tj) - tj);
            if (2.0 * HalfDistance(tj, ta, jmax) < distance_) lo = mid;
            else hi = mid;
        }
        peak_velocity_ = lo;
        tj_ = std::min(amax / jmax, std::sqrt(peak_velocity_ / jmax));
        ta_ = peak_velocity_ / (jmax * tj_) - tj_;
        tv_ = 0.0;
    }
    peak_velocity_ = HalfVelocity(tj_, ta_, jmax);
    half_distance_ = HalfDistance(tj_, ta_, jmax);
    total_time_ = 2.0 * (2.0 * tj_ + ta_) + tv_;
    valid_ = true;
    return true;
}

ScurveState ScurveProfile::SamplePositive(double t) const
{
    // Integrating constant jerk over each phase gives position, velocity, and
    // acceleration without allocating or depending on a real-time clock.
    const double t1=tj_, t2=t1+ta_, t3=t2+tj_, t4=t3+tv_;
    const double t5=t4+tj_, t6=t5+ta_, t7=t6+tj_;
    double x=0,v=0,a=0;
    const double j=jmax_;
    const double times[7]={t1,t2,t3,t4,t5,t6,t7};
    const double jerks[7]={j,0,-j,0,-j,0,j};
    double begin=0;
    for(int i=0;i<7;++i) {
        const double end=times[i];
        if(t <= end) {
            const double h=std::max(0.0,t-begin);
            x += v*h + 0.5*a*h*h + jerks[i]*h*h*h/6.0;
            v += a*h + 0.5*jerks[i]*h*h;
            a += jerks[i]*h;
            return {x,v,a,false};
        }
        const double h=end-begin;
        x += v*h + 0.5*a*h*h + jerks[i]*h*h*h/6.0;
        v += a*h + 0.5*jerks[i]*h*h;
        a += jerks[i]*h; begin=end;
    }
    return {distance_,0.0,0.0,true};
}

ScurveState ScurveProfile::Sample(double time) const
{
    if (!valid_) return {0,0,0,true};
    if (time <= 0) return {0,0,0,false};
    if (time >= total_time_) return {sign_*distance_,0,0,true};
    ScurveState s=SamplePositive(time); s.position*=sign_; s.velocity*=sign_;
    s.acceleration*=sign_; return s;
}

ScurveState ScurveProfile::Update(double dt)
{
    if (dt > 0 && valid_) elapsed_ += dt;
    if (!valid_ || elapsed_ > total_time_) elapsed_ = total_time_;
    return Sample(elapsed_);
}

bool ValidateScurveLimits(double amax,double jmax,double b0,double wc,double volts)
{
    // The 0.7 headroom rule keeps requested acceleration below the nominal
    // plant authority, while jmax/amax <= wc avoids an excessively abrupt ramp.
    if (!(b0 > 0.0) || !std::isfinite(b0)) return false;
    return amax/(b0*volts) <= 0.7 && jmax/amax <= wc;
}
