#include "tuning/tuning.h"
#include <cmath>

ControlMode currentMode = ControlMode::PID;
PIDParams pid_cfg;
LADRCParams ladrc_cfg;

AccelConfig accel_cfg;

double driver_deadzone = 10.0;
double driver_exponent = 2.0;

double get_curved_input(double input) {
    if (std::abs(input) < driver_deadzone) return 0.0;
    
    double sign = (input > 0) ? 1.0 : -1.0;
    double scaled = (std::abs(input) - driver_deadzone) / (100.0 - driver_deadzone);
    return sign * (std::pow(scaled, driver_exponent) * 100.0);
}
