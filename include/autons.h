#ifndef AUTONS_H
#define AUTONS_H

// ============================================================================
// Autonomous Routines & Selector Declarations Header
// ============================================================================
// Defines the available autonomous routines (Right Side, Left Side, Skills),
// the selection enumeration used across the controller UI and competition callbacks,
// and hardware/odometry initialization helpers executed during pre-auton.
//
// All motion commands:
//   - drive(distance_in, speed_pct, timeout_s)
//   - turn(angle_deg, speed_pct, timeout_s)
//   - turn_to(heading_deg, speed_pct, timeout_s)
//   - bdrive(forward_in, sideways_in, end_heading_deg, speed_pct, timeout_s)
//   - bezier_drive(...)
//   - sdrive()
// are accessible by including this header.
// ============================================================================

#include "motion.h" // Include autonomous motion commands (drive, turn, bdrive, sdrive)

// Enumeration of selectable autonomous routines
enum class AutonSelection {
    None,       // Default state (dispatches to RightSide)
    RightSide,  // Right-side match autonomous routine
    LeftSide,   // Left-side match autonomous routine
    Skills      // Full Programming Skills autonomous routine
};

// Global autonomous selection variable tracking user choice
extern AutonSelection g_selected_auton;

// Global flag tracking whether IMU calibration timed out during pre_auton()
extern bool g_imu_calibration_failed;

// Re-initializes and tares odometry, parallel/perpendicular tracking wheels, and IMU heading
void reset_odometry_and_sensors();

// Right Side Match Autonomous routine (distances in inches, angles in degrees)
void RS();

// Left Side Match Autonomous routine (distances in inches, angles in degrees)
void LS();

// Programming Skills Autonomous routine (distances in inches, angles in degrees)
void Skills();

// Master autonomous function invoked by the VEX Competition system
void autonomous();

#endif // AUTONS_H
