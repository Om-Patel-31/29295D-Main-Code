#include "tuning.h"
#include "config.h"

// Driver tuning is deliberately separate from autonomous control. These
// helpers use VEX sensor types and are therefore kept out of control modules.
using namespace vex;

// Tune these five driver profiles independently for the robot. The first
// value scales forward/reverse acceleration and the second scales turning.
static const DriverGains kDriverGains[] = {
    // Each entry is forward/turn scaling; names are shown by the selector UI.
    {0.12, 0.12, "Cubic"},
    {0.12, 0.16, "Quintic"},
    {0.16, 0.12, "Quintic with Trigonometric bounds"},
    {0.16, 0.16, "Trigonometric Quintic"},
    {0.20, 0.20, "Trigonometric with quintic bounds"}
};

const DriverGains& driver_gains(DriverGraph graph)
{
    // The enum order is the table index used by the Brain selector.
    return kDriverGains[static_cast<int>(graph)];
}

double mm_per_motor_degree()
{
    return config::wheel_diameter_mm * 3.141592653589793 / 360.0 *
           config::drive_gear_ratio;
}

double average_side(motor& front, motor& rear)
{
    return 0.5 * (front.position(degrees) + rear.position(degrees)) *
           mm_per_motor_degree();
}

double heading_bias(inertial& imu)
{
    // 100 samples at 10 ms span about 1.0 s and reduce startup sensor noise.
    double sum = 0.0;
    for (int sample = 0; sample < 100; ++sample) {
        sum += imu.heading(degrees) * 3.141592653589793 / 180.0;
        wait(10, msec);
    }
    return sum / 100.0;
}
