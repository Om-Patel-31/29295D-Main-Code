#ifndef TUNING_H
#define TUNING_H

// VEX-facing driver-control tuning interface. DriverGraph selects a response
// profile; gains are dimensionless scalars applied to controller percent input.
#include "vex.h"

enum class DriverGraph {
    Cubic,
    Quintic,
    QuinticTrigonometricBounds,
    TrigonometricQuintic,
    TrigonometricQuinticBounds
};

struct DriverGains {
    // These gains scale forward and turn joystick values before voltage output.
    double acceleration_gain;
    double turning_gain;
    const char* name;
};

double mm_per_motor_degree();
// Converts one motor degree to wheel travel in mm using configured geometry.
double average_side(vex::motor& front, vex::motor& rear);
// Reads two motor positions and returns their average travel in mm.
double heading_bias(vex::inertial& imu);
// Averages 100 inertial readings to estimate startup heading bias in rad.
const DriverGains& driver_gains(DriverGraph graph);

#endif
