#include "tuning.h" // Include header containing the RobotTuning class declaration

// ============================================================================
// ALL ROBOT TUNING VARIABLES - DECLARATION & INITIALIZATION
// ============================================================================
// ALL TUNING VALUES FOR THE ENTIRE ROBOT ARE CONSOLIDATED RIGHT HERE AT THE TOP.
// Simply edit the values below to adjust any aspect of the robot's performance!
// Units used: Inches, Degrees, Seconds, Volts, and Percent.
// ============================================================================

RobotTuning tuning = {
    // ---- 1. Drivetrain Geometry (Inches) ----
    3.25,                           // wheel_diameter_in: Driven wheel diameter in inches
    0.75,                           // drive_gear_ratio: Wheel revs per motor shaft rev (e.g., 36T:48T = 0.75)
    13.5,                           // track_width_in: Center-to-center distance between left and right wheels
    2.0,                            // tracking_wheel_diameter_in: Unpowered tracking wheel diameter in inches
    2.0,                            // horizontal_wheel_offset_in: Perpendicular wheel distance from center of rotation
    false,                          // use_horizontal_tracking_wheel: Set to true once horizontal wheel is installed

    // ---- 2. Smart Port Wiring Configuration & Motor Directions ----
    2,                              // left_front_port: Smart port for front-left drive motor
    12,                             // left_rear_port: Smart port for rear-left drive motor
    1,                              // right_front_port: Smart port for front-right drive motor
    11,                             // right_rear_port: Smart port for rear-right drive motor
    true,                           // left_front_reversed: Direction reversal flag for front-left drive motor (true = reversed)
    true,                           // left_rear_reversed: Direction reversal flag for rear-left drive motor (true = reversed)
    false,                          // right_front_reversed: Direction reversal flag for front-right drive motor (false = normal)
    false,                          // right_rear_reversed: Direction reversal flag for rear-right drive motor (false = normal)
    3,                              // lift_left_port: Smart port for left cascade lift motor (normal)
    4,                              // lift_right_port: Smart port for right cascade lift motor (reversed)
    5,                              // claw_port: Smart port for intake/claw motor
    14,                             // tracking_parallel_port: Smart port for parallel rotation tracking wheel
    17,                             // tracking_perp_port: Smart port for lateral rotation tracking wheel
    21,                             // vision_sensor_port: Smart port for AI Vision sensor (assigned to port 21 to avoid conflict)
    20,                             // imu_port: Smart port for 3-Axis V5 Inertial sensor

    // ---- 3. Driver Control Gains ----
    0.25,                           // driver_accel_gain: Scales Axis 3 (-100..+100%) to +/-12 Volts
    0.17,                           // driver_turn_gain: Scales Axis 1 (-100..+100%) to +/-12 Volts

    // ---- 4. Linear LADRC & ESO Parameters (Drive) ----
    45.0,                           // drive_b0: Linear acceleration control authority (in/s^2 per Volt)
    0.65,                            // drive_settle_time: Desired settling time for position loop in seconds
    5.0,                            // drive_wo_ratio: Ratio of observer speed to controller speed (wo / wc)
    300.0,                          // drive_max_disturb: Maximum disturbance acceleration clamp (in/s^2)

    // ---- 5. Angular LADRC & ESO Parameters (Turn) ----
    400.0,                          // turn_b0: Rotational acceleration control authority (deg/s^2 per Volt)
    0.5,                            // turn_settle_time: Desired settling time for heading loop in seconds
    5.0,                            // turn_wo_ratio: Ratio of observer speed to controller speed (wo / wc)
    3000.0,                         // turn_max_disturb: Maximum disturbance angular acceleration clamp (deg/s^2)

    // ---- 6. Straight & Curved Drive Quintic Profile Limits ----
    50.0,                           // drive_max_speed: Maximum cruise speed for linear paths (in/s)
    55.0,                           // drive_max_accel: Maximum linear acceleration (in/s^2)
    160.0,                          // drive_max_jerk: Bounded peak jerk rating for profile diagnostics (in/s^3)
    4.0,                            // min_turn_radius_in: Minimum allowable radius for Bezier curves (inches)
    80.0,                           // max_lateral_accel: Maximum allowable lateral centripetal acceleration (in/s^2)

    // ---- 7. Turn-In-Place Quintic Profile Limits ----
    300.0,                          // turn_max_speed: Maximum angular velocity for turns in place (deg/s)
    600.0,                          // turn_max_accel: Maximum angular acceleration for turns (deg/s^2)
    2400.0,                         // turn_max_jerk: Bounded peak angular jerk rating for profile diagnostics (deg/s^3)

    // ---- 8. Cross-Track Path Following Correction ----
    3.0,                            // cross_track_gain: Steering correction in degrees per inch of lateral error
    15.0,                           // cross_track_max: Maximum clamp on steering correction angle (degrees)

    // ---- 9. Autonomous Motion Exit & Settle Criteria ----
    0.5,                            // drive_tolerance_in: Distance error threshold to declare path done (inches)
    1.0,                            // turn_tolerance_deg: Heading error threshold to declare turn done (degrees)
    2.0,                            // settle_speed_in_s: Maximum linear speed to confirm robot is stopped (in/s)
    8.0,                            // settle_turn_rate_deg_s: Maximum angular rate to confirm robot stopped (deg/s)
    0.10,                           // settle_hold_time_s: Required time within tolerances to finish move (seconds)
    2.0,                            // motion_timeout_margin_s: Safety timeout added to planned duration (seconds)
    true,                           // chain_motions: true = no brake/settle between moves (only sdrive() brakes)

    // ---- 10. Manipulator Tuning (Lift & Claw) ----
    2.0,                            // lift_kp: Proportional gain for cascade lift position PID
    0.0,                            // lift_ki: Integral gain for cascade lift position PID
    0.1,                            // lift_kd: Derivative gain for cascade lift position PID
    5.0,                            // lift_integral_max: Anti-windup limit for lift PID integrator (Volts)
    0.1,                            // lift_settle_error_in: Position error threshold for lift motion (inches)
    80.0,                           // claw_close_power_pct: Motor power percentage when clamping claw
    -80.0,                          // claw_open_power_pct: Motor power percentage when opening claw

    // ---- 11. System Hardware Limits ----
    12.0,                           // max_voltage: Maximum allowable motor command voltage (Volts)
    100.0,                          // control_hz: Control loop update frequency (Hz)
    5.0                             // imu_calib_timeout_s: Inertial sensor calibration timeout (seconds)
};

// ============================================================================
// Encoder Geometry Helper Functions
// ============================================================================

// Computes linear distance traveled in inches for one degree of motor rotation
double inches_per_motor_degree()
{
    // Distance = (pi * wheel_diameter) / 360 degrees * external gear ratio
    const double pi = 3.14159265358979323846; // Ratio of circle circumference to diameter
    return (tuning.wheel_diameter_in * pi / 360.0) * tuning.drive_gear_ratio; // Return inches per degree
}

// Computes linear distance traveled in millimeters for one degree of motor rotation
double mm_per_motor_degree()
{
    // Convert inches per motor degree directly to millimeters (1 inch = 25.4 mm)
    return inches_per_motor_degree() * 25.4; // Return millimeters per degree
}
