#ifndef ROBOT_H
#define ROBOT_H

#include "vex.h"     // Include VEX Robotics V5 API
#include "odometry.h" // Include Odometry class definition

// ============================================================================
// Robot Hardware Declarations Header
// ============================================================================
// Provides extern declarations for all physical motors, sensors, and controllers
// configured in robot.cpp.

// V5 Brain instance
extern vex::brain Brain;

// Primary handheld controller instance
extern vex::controller Controller;

// Drivetrain motors
extern vex::motor left_front;  // Front-left chassis motor
extern vex::motor left_rear;   // Rear-left chassis motor
extern vex::motor right_front; // Front-right chassis motor
extern vex::motor right_rear;  // Rear-right chassis motor

// Manipulator motors
extern vex::motor lift_left;   // Left cascade lift motor
extern vex::motor lift_right;  // Right cascade lift motor
extern vex::motor claw;        // Mobile goal / game piece claw motor

// Tracking wheel rotation sensors
extern vex::rotation tracking_parallel; // Parallel forward tracking wheel
extern vex::rotation tracking_perp;     // Perpendicular lateral tracking wheel

// AI Vision sensor
extern vex::aivision vision_sensor;     // AI Vision sensor

// V5 3-Axis Inertial Sensor (IMU)
extern vex::inertial imu;               // Inertial gyroscope sensor

// Global odometry tracking instance
extern Odometry g_odometry;             // Odometry pose estimator

// Helper functions for reading drivetrain wheel distances
double average_left_degrees();  // Average rotation of left drivetrain motors (degrees)
double average_right_degrees(); // Average rotation of right drivetrain motors (degrees)
double average_left_inches();   // Average linear distance of left wheels (inches)
double average_right_inches();  // Average linear distance of right wheels (inches)
double average_left_mm();       // Average linear distance of left wheels (millimeters)
double average_right_mm();      // Average linear distance of right wheels (millimeters)

#endif // ROBOT_H

