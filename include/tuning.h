#ifndef TUNING_H
#define TUNING_H

#include <cmath> // Include math library for fabs, sqrt, sin, cos

// ============================================================================
// Robot & Control System Tuning Configuration Class
// ============================================================================
// Contains all user-tunable constants for the entire robot in one unified class:
//   - Physical drivetrain geometry (inches)
//   - Smart port wiring mappings
//   - Driver control gains & joystick scaling
//   - LADRC & Extended State Observer (ESO) disturbance rejection parameters
//   - Quintic minimum-jerk motion profile constraints (speed, acceleration)
//   - Cross-track path following correction gains
//   - Motion completion tolerances and settling criteria
//   - Manipulator PID and power constants
//   - Hardware system safety limits
//
// All user-facing values are entered in standard competition units:
//   Inches, Degrees, Seconds, Volts, and Percent.
// Inline conversion helpers convert to internal SI units (mm, rad) automatically.
// ============================================================================
class RobotTuning {
public:
    // ---- 1. Drivetrain Geometry (Inches) ----
    double wheel_diameter_in;           // Driven wheel diameter (inches)
    double drive_gear_ratio;            // External gear ratio (wheel rev / motor rev)
    double track_width_in;              // Distance between left and right wheel contact patches (inches)
    double tracking_wheel_diameter_in;  // Odometry tracking wheel diameter (inches)
    double horizontal_wheel_offset_in;  // Perpendicular tracking wheel distance from center of rotation (inches)
    bool use_horizontal_tracking_wheel; // Enable/disable lateral tracking wheel in odometry calculations

    // ---- 2. Smart Port Wiring Configuration & Motor Directions ----
    int left_front_port;        // Smart port for front-left drive motor
    int left_rear_port;         // Smart port for rear-left drive motor
    int right_front_port;       // Smart port for front-right drive motor
    int right_rear_port;        // Smart port for rear-right drive motor
    bool left_front_reversed;   // Direction reversal flag for front-left drive motor (true = reversed, false = normal)
    bool left_rear_reversed;    // Direction reversal flag for rear-left drive motor (true = reversed, false = normal)
    bool right_front_reversed;  // Direction reversal flag for front-right drive motor (true = reversed, false = normal)
    bool right_rear_reversed;   // Direction reversal flag for rear-right drive motor (true = reversed, false = normal)
    int lift_left_port;         // Smart port for left cascade lift motor
    int lift_right_port;        // Smart port for right cascade lift motor
    int claw_port;              // Smart port for intake/claw motor
    int tracking_parallel_port; // Smart port for forward/parallel rotation tracking sensor
    int tracking_perp_port;     // Smart port for lateral/perpendicular rotation tracking sensor
    int vision_sensor_port;     // Smart port for AI Vision sensor
    int imu_port;               // Smart port for 3-Axis V5 Inertial sensor

    // ---- 3. Driver Control Gains ----
    double driver_accel_gain; // Forward/reverse joystick sensitivity scaling (Axis 3)
    double driver_turn_gain;  // Steering joystick sensitivity scaling (Axis 1)

    // ---- 4. Linear LADRC & ESO Parameters (Drive/Longitudinal) ----
    double drive_b0;          // Drivetrain linear acceleration authority (in/s^2 per Volt)
    double drive_settle_time; // Desired settling time for position loop (seconds)
    double drive_wo_ratio;    // Ratio of observer bandwidth to controller bandwidth (wo / wc)
    double drive_max_disturb; // Maximum disturbance acceleration cap to prevent observer windup (in/s^2)

    // ---- 5. Angular LADRC & ESO Parameters (Turn/Heading) ----
    double turn_b0;           // Drivetrain rotational acceleration authority (deg/s^2 per Volt)
    double turn_settle_time;  // Desired settling time for heading loop (seconds)
    double turn_wo_ratio;     // Ratio of observer bandwidth to controller bandwidth (wo / wc)
    double turn_max_disturb;  // Maximum disturbance rotational acceleration cap (deg/s^2)

    // ---- 6. Straight & Curved Drive Quintic Profile Limits ----
    double drive_max_speed;    // Maximum cruise velocity for straight/curved drives (in/s)
    double drive_max_accel;    // Maximum acceleration for straight/curved drives (in/s^2)
    double drive_max_jerk;     // Bounded peak jerk rating for profile diagnostics (in/s^3)
    double min_turn_radius_in; // Minimum allowable turn radius for Bezier curves (inches)
    double max_lateral_accel;  // Maximum allowable centripetal lateral acceleration on curves (in/s^2)

    // ---- 7. Turn-In-Place Quintic Profile Limits ----
    double turn_max_speed;     // Maximum angular velocity for turns (deg/s)
    double turn_max_accel;     // Maximum angular acceleration for turns (deg/s^2)
    double turn_max_jerk;      // Bounded peak angular jerk rating for profile diagnostics (deg/s^3)

    // ---- 8. Cross-Track Path Following Correction ----
    double cross_track_gain;   // Heading correction steering gain (degrees per inch of lateral offset)
    double cross_track_max;    // Maximum steering correction limit clamp (degrees)

    // ---- 9. Autonomous Motion Exit & Settle Criteria ----
    double drive_tolerance_in;     // Target distance tolerance for declaring path finished (inches)
    double turn_tolerance_deg;     // Target heading tolerance for declaring turn finished (degrees)
    double settle_speed_in_s;      // Velocity threshold below which robot is considered stopped (in/s)
    double settle_turn_rate_deg_s; // Turn rate threshold below which robot is considered stopped (deg/s)
    double settle_hold_time_s;     // Required duration within tolerance to declare completion (seconds)
    double motion_timeout_margin_s;// Extra safety margin added to theoretical profile duration (seconds)
    bool chain_motions;            // true = moves hand off without braking/settling; only sdrive() brakes

    // ---- 10. Manipulator Tuning (Lift & Claw) ----
    double lift_kp;              // Proportional gain for cascade lift position PID
    double lift_ki;              // Integral gain for cascade lift position PID
    double lift_kd;              // Derivative gain for cascade lift position PID
    double lift_integral_max;    // Anti-windup clamp for lift integral term (Volts)
    double lift_settle_error_in; // Lift target height tolerance threshold (inches)
    double claw_close_power_pct; // Motor power percentage when closing claw
    double claw_open_power_pct;  // Motor power percentage when opening claw

    // ---- 11. System Hardware Limits ----
    double max_voltage;         // Maximum allowable motor command voltage (Volts, normally 12.0)
    double control_hz;          // Nominal control loop execution rate (Hz)
    double imu_calib_timeout_s; // Inertial sensor calibration timeout (seconds)

    // ---- Inline Unit Conversion Helpers (Inches/Deg -> Internal mm/rad) ----
    double wheel_diameter_mm() const { return wheel_diameter_in * 25.4; } // Convert wheel diameter to mm
    double track_width_mm() const { return track_width_in * 25.4; }       // Convert track width to mm
    double horizontal_wheel_offset_mm() const { return horizontal_wheel_offset_in * 25.4; } // Convert offset to mm
    double tracking_wheel_circ_mm() const { return tracking_wheel_diameter_in * 25.4 * 3.14159265358979323846; } // Circumference in mm

    // LADRC bandwidth conversions: wc = 4.0 / settling_time
    double drive_wc() const { return 4.0 / drive_settle_time; }                         // Closed-loop bandwidth in rad/s
    double drive_wo() const { return drive_wc() * drive_wo_ratio; }                     // Observer bandwidth in rad/s
    double drive_b0_mm() const { return drive_b0 * 25.4; }                              // b0 in (mm/s^2)/V
    double drive_max_disturb_mm() const { return drive_max_disturb * 25.4; }            // Disturbance clamp in mm/s^2

    double turn_wc() const { return 4.0 / turn_settle_time; }                           // Closed-loop bandwidth in rad/s
    double turn_wo() const { return turn_wc() * turn_wo_ratio; }                       // Observer bandwidth in rad/s
    double turn_b0_rad() const { return turn_b0 * (3.14159265358979323846 / 180.0); }   // b0 in (rad/s^2)/V
    double turn_max_disturb_rad() const { return turn_max_disturb * (3.14159265358979323846 / 180.0); } // Clamp in rad/s^2

    // Motion limits in mm / rad
    double v_max_mm_s() const { return drive_max_speed * 25.4; }                       // Max velocity in mm/s
    double a_max_mm_s2() const { return drive_max_accel * 25.4; }                      // Max acceleration in mm/s^2
    double j_max_mm_s3() const { return drive_max_jerk * 25.4; }                       // Max jerk in mm/s^3
    double min_turn_radius_mm() const { return min_turn_radius_in * 25.4; }            // Min radius in mm
    double lateral_accel_max_mm() const { return max_lateral_accel * 25.4; }           // Lateral accel in mm/s^2

    double turn_v_max_rad_s() const { return turn_max_speed * (3.14159265358979323846 / 180.0); }   // Max turn speed in rad/s
    double turn_a_max_rad_s2() const { return turn_max_accel * (3.14159265358979323846 / 180.0); }  // Max turn accel in rad/s^2
    double turn_j_max_rad_s3() const { return turn_max_jerk * (3.14159265358979323846 / 180.0); }   // Max turn jerk in rad/s^3

    double cross_track_gain_rad_per_mm() const {
        return (cross_track_gain * (3.14159265358979323846 / 180.0)) / 25.4;          // Cross-track gain in rad/mm
    }
    double cross_track_max_rad() const {
        return cross_track_max * (3.14159265358979323846 / 180.0);                    // Max cross-track clamp in rad
    }

    double drive_tolerance_mm() const { return drive_tolerance_in * 25.4; }            // Linear tolerance in mm
    double turn_tolerance_rad() const { return turn_tolerance_deg * (3.14159265358979323846 / 180.0); } // Angular tolerance in rad
    double settle_speed_mm_s() const { return settle_speed_in_s * 25.4; }              // Settle speed in mm/s
    double settle_turn_rate_rad_s() const { return settle_turn_rate_deg_s * (3.14159265358979323846 / 180.0); } // Settle rate in rad/s
};

// Global instance of the tuning configuration, declared and initialized at the top of tuning.cpp
extern RobotTuning tuning;

// Encoder calculation helper declarations
double inches_per_motor_degree(); // Calculates linear inches traveled per motor degree
double mm_per_motor_degree();     // Calculates linear millimeters traveled per motor degree

#endif // TUNING_H
