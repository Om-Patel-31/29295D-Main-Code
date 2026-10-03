#include "include/robot.h"

vex::brain Brain;
vex::controller Controller;

vex::motor LFM = vex::motor(vex::PORT11, vex::ratio6_1, true);
vex::motor LBM = vex::motor(vex::PORT12, vex::ratio6_1, true);
vex::motor RFM = vex::motor(vex::PORT20, vex::ratio6_1, false);
vex::motor RBM = vex::motor(vex::PORT19, vex::ratio6_1, false);

