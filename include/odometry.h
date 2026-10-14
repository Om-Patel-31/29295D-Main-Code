#ifndef ODOMETRY_H
#define ODOMETRY_H

// Midpoint differential-drive odometry. Pose coordinates are mm and heading is
// rad; wheel distances are cumulative mm and inertial rate is rad/s.

struct Pose2
{
    // x and y use the fixed field frame; heading is the inertial heading after
    // subtracting the startup bias.
    double x;
    double y;
    double heading;
};

struct OdometryMeasurement
{
    // left/right/vertical/horizontal are cumulative distances in mm.
    // inertial_heading is rad and inertial_rate is rad/s.
    double left_mm;
    double right_mm;
    double vertical_mm;
    double horizontal_mm;
    double inertial_heading;
    double inertial_rate;
};

class Odometry
{
public:
    // horizontal_wheel_offset is the lateral tracking-wheel offset in mm and
    // corrects sideways displacement caused by rotation.
    explicit Odometry(double horizontal_wheel_offset = 0.0);
    void Reset(const OdometryMeasurement& measurement, double heading_bias = 0.0);
    Pose2 Update(double dt, const OdometryMeasurement& measurement);
    Pose2 pose() const { return pose_; }
    double velocity() const { return velocity_; }
    double turn_rate() const { return turn_rate_; }

private:
    // previous_ stores the last cumulative sensor sample. The midpoint angle
    // is used so each incremental translation is rotated at the interval mean.
    Pose2 pose_;
    OdometryMeasurement previous_;
    double heading_bias_;
    double horizontal_offset_mm_; // perpendicular-wheel offset used in Update()
    double velocity_, turn_rate_;

    bool initialized_;
};

#endif
