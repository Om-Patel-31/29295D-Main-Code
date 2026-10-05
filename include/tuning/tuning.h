#ifndef TUNING_H
#define TUNING_H

#include "vex.h"

// Global Mode Setting
// true = Tuning Mode (Graphics active), false = Match Mode (Graphics disabled)
constexpr bool TUNING_MODE = true;

// Acceleration Tuning Config
struct AccelConfig {
    bool show_accel = true;
    bool show_vel = true;
    bool separate_sides = false;
    double smoothing_factor = 0.2; // EMA alpha: 0.0 to 1.0 (higher = less filtered)
    double y_axis_max = 50.0;
    int update_rate_ms = 50;
    int history_length = 100;
};

extern AccelConfig accel_cfg;

// Control Mode Selection
enum class ControlMode {
    PID,
    LADRC
};

extern ControlMode currentMode;

// Tuning Parameters
struct PIDParams {
    double kp = 0.5;
    double ki = 0.0;
    double kd = 0.1;
};

struct LADRCParams {
    double bandwidth = 10.0;
    double observer_bandwidth = 20.0;
};

extern PIDParams pid_cfg;
extern LADRCParams ladrc_cfg;

// Curve Parameters
extern double driver_deadzone;
extern double driver_exponent;

// Function to get curved input (implemented in tuning.cpp)
double get_curved_input(double input);

#endif