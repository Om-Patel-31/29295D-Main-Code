#ifndef ROBOT_H
#define ROBOT_H

#include "vex.h"

void initialize_robot();

extern brain Brain;
extern controller Controller;
extern motor LFM;
extern motor LBM;
extern motor RFM;
extern motor RBM;
struct DriverTuning{float deadzone; float response_exponent; float acceleration_gain; float turn_gain};
extern DriverTuning driver_tuning;

#endif