#include "odometry.h"
#include <cmath>

// Differential-drive increments are rotated by the midpoint heading. Any
// non-finite sensor value is rejected so a bad sample cannot corrupt the pose.

// When true, forward distance comes from the parallel tracking wheel (vertical_mm).
// When false, the average of the left/right drive encoder differences is used.
// Set to false until the parallel tracking wheel is physically installed.
static const bool kUseParallelWheelForward = false;

Odometry::Odometry(double horizontal_wheel_offset_mm)
    : pose_{0,0,0}, previous_{0,0,0,0,0,0},
      heading_bias_(0.0), horizontal_offset_mm_(horizontal_wheel_offset_mm),
      velocity_(0), turn_rate_(0), initialized_(false) {}

void Odometry::Reset(const OdometryMeasurement& m, double bias)
{
    // The inertial reading at reset establishes the field-frame heading offset.
    if (!std::isfinite(m.left_mm) || !std::isfinite(m.right_mm) ||
        !std::isfinite(m.vertical_mm) || !std::isfinite(m.horizontal_mm) ||
        !std::isfinite(m.inertial_heading) || !std::isfinite(m.inertial_rate) ||
        !std::isfinite(bias)) {
        return;
    }
    previous_=m; heading_bias_=bias; pose_={0,0,m.inertial_heading-bias};
    velocity_=0; turn_rate_=0; initialized_=true;
}
Pose2 Odometry::Update(double dt, const OdometryMeasurement& m)
{
    // Sensor distances are cumulative, so each update uses only the newest
    // left/right and tracking-wheel differences.
    if (!std::isfinite(m.left_mm) || !std::isfinite(m.right_mm) ||
        !std::isfinite(m.vertical_mm) || !std::isfinite(m.horizontal_mm) ||
        !std::isfinite(m.inertial_heading) || !std::isfinite(m.inertial_rate)) {
        return pose_;
    }
    if(!initialized_){Reset(m,0);return pose_;}
    if(!(dt>0 && std::isfinite(dt)))return pose_;
    double dl=m.left_mm-previous_.left_mm, dr=m.right_mm-previous_.right_mm;
    // Forward distance: tracking wheel when installed, drive encoders otherwise.
    double dv = m.vertical_mm - previous_.vertical_mm;
    double forward = kUseParallelWheelForward ? dv : 0.5*(dl+dr);
    double dtheta=m.inertial_heading-previous_.inertial_heading; // radians
    // dh is the raw horizontal-wheel displacement this tick.
    // Subtracting horizontal_offset_mm_ * dtheta removes the false lateral
    // displacement caused by the robot rotating about a point offset from the wheel.
    double dh = m.horizontal_mm - previous_.horizontal_mm;
    double sideways = dh - horizontal_offset_mm_ * dtheta;
    double mid=pose_.heading+0.5*dtheta;
    pose_.x+=forward*std::cos(mid)-sideways*std::sin(mid);
    pose_.y+=forward*std::sin(mid)+sideways*std::cos(mid);
    pose_.heading=m.inertial_heading-heading_bias_; velocity_=forward/dt; turn_rate_=m.inertial_rate;
    previous_=m; return pose_;
}
