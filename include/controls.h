#ifndef CONTROLS_H
#define CONTROLS_H

#include "motion.h"

// ============================================================
// POSE ACCESS FUNCTIONS
// ============================================================

Pose get_pose();

float get_x();

float get_y();

float get_heading();


// ============================================================
// AUTONOMOUS MOTION CONTROL
// ============================================================

// Basic straight-line motion
void move_forward(float distance, float duration);

void move_forward_pid(float distance, float duration);

// Basic turn-in-place motion
void turn_radians(float angle, float duration);

void turn_degrees(float angle, float duration);

// Cubic Bezier curve following
void follow_bezier(
	float x0,
	float y0,
	float x1,
	float y1,
	float x2,
	float y2,
	float x3,
	float y3,
	float duration
);


// ============================================================
// DRIVER CONTROL
// ============================================================

// Driver-controlled drive with acceleration limiting and deadzone
void driver_drive(float forward, float turn);

void reset_driver();


// ============================================================
// SYSTEM INITIALIZATION
// ============================================================

void initialize_motion_control();


#endif // CONTROLS_H