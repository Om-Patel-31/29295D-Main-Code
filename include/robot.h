#ifndef ROBOT_H
#define ROBOT_H

#include "vex.h"

void initialize_robot();
void move_to(double targetX, double targetY);
void update_odometry();

extern vex::brain Brain;
extern vex::controller Controller;
extern vex::motor LFM;
extern vex::motor LBM;
extern vex::motor RFM;
extern vex::motor RBM;
extern vex::rotation VRS;
extern vex::rotation HRS;
extern vex::inertial IMU;

#endif