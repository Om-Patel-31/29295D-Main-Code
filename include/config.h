#ifndef CONFIG_H
#define CONFIG_H

// Central configuration for geometry, controller gains, motion limits, and
// timeout policy. Lengths are mm, rates are rad/s or rad, acceleration is
// mm/s^2, voltage is V, and time is s. This header has no VEX dependency.
#include <cmath>

namespace config
{
// Geometry values are TODO placeholders. Measure every value on the robot
// before tuning because these values convert motor measurements into mm.
static const double wheel_diameter_mm = 69.85; // TODO: driven wheel diameter
static const double drive_gear_ratio = 1.0;    // TODO: wheel rev / motor rev
static const double track_width_mm = 400.0;    // TODO: left-right wheel spacing
static const double vertical_wheel_diameter_mm = 50.8; // TODO
static const double horizontal_wheel_diameter_mm = 50.8; // TODO
// TODO: measure horizontal_wheel_offset_mm on the real robot. This is the
// perpendicular distance (mm) from the robot's rotation center to the
// horizontal tracking wheel contact patch. A mock value of 0.0 is used
// until measured.
static const double horizontal_wheel_offset_mm = 0.0; // TODO: measure

// Tracking wheel geometry. Wheel is 2" diameter (50.8 mm).
static const double tracking_wheel_circumference_mm = 50.8 * M_PI;

// Smart port assignments — mock values; update once hardware is wired.
static const int tracking_parallel_port = 6;  // TODO: confirm port
static const int tracking_perp_port     = 7;  // TODO: confirm port
static const int vision_sensor_port     = 11; // TODO: confirm port

// Lift winch geometry for a VEX cascade lift.
// Winch spool is a VEX HS 36-tooth gear (DP24, pitch diameter = 1.5 in).
// Circumference = pi * 1.5 in.
// lift_total_reduction: motor output shaft rev / lift shaft rev (no external
// gearing; 6:1 blue cartridge is accounted for by the motor encoder natively).
static const double lift_total_reduction         = 1.0;   // TODO: verify no external gearing
static const double lift_winch_pitch_diameter_in = 1.5;   // VEX HS 36T gear, DP24
// Number of moving stages in the cascade lift. Each stage adds one winch
// circumference of linear travel per motor revolution.
// TODO: set to the actual number of moving stages on the robot.
static const int    lift_cascade_stages          = 1;     // TODO: set to actual stage count

// Lift PID gains — TODO: tune on the robot.
static const double lift_kp = 2.0;  // TODO: tune
static const double lift_ki = 0.0;  // TODO: tune
static const double lift_kd = 0.1;  // TODO: tune
static const double lift_integral_max    = 5.0;  // V, anti-windup clamp
static const double lift_settle_error_in = 0.1;  // in, exit threshold

// Claw timed open-loop constants.
// ASSUMPTION: positive power (percent) closes the claw. Verify on hardware.
static const double claw_close_power_pct =  80.0; // TODO: tune
static const int    claw_close_time_ms   = 600;   // TODO: tune
static const double claw_open_power_pct  = -80.0; // TODO: tune
static const int    claw_open_time_ms    = 500;   // TODO: tune

// Lift target heights for scoring macros — TODO: measure on field.
static const double lift_height_pins_in  = 8.0;   // TODO: measure
static const double lift_height_cups_in  = 5.0;   // TODO: measure

// Minimum time between macro triggers in ms.
static const double macro_cooldown_ms = 2000.0;


// LADRC plant gains are TODO calibration values in mm/s^2 per V for translation
// and rad/s^2 per V for heading. Autonomous refuses to run while either b0 is 0.
static const double longitudinal_b0 = 0.0; // TODO: identify from voltage step
// Heading b0 uses U_turn (the heading-channel voltage); the mixer applies
// V_R - V_L = 2*U_turn, so do not divide this calibration by two again.
static const double heading_b0 = 0.0;       // TODO: identify in rad/s^2 per U_turn
static const double longitudinal_wc = 4.0;    // rad/s
static const double heading_wc = 4.0;         // rad/s
static const double observer_bandwidth_ratio = 4.0; // wo / wc

// Motion limits constrain the reference profile; they are not measured motor
// capabilities and should be checked against the robot before competition.
static const double v_max_mm_s = 600.0;       // mm/s
static const double a_max_mm_s2 = 300.0;      // mm/s^2
static const double j_max_mm_s3 = 1200.0;     // mm/s^3
static const double max_voltage = 12.0;       // V
static const double control_hz = 100.0;       // Hz, scheduling target only

// Path and follower limits. The tolerances and acceleration limit are TODO
// robot-tuning values; the timeout values are safety limits in seconds.
static const double min_turn_radius_mm = 300.0; // mm
static const double lateral_heading_gain = 0.0; // 1/s, optional and off
static const double lateral_acceleration_max = 2000.0; // mm/s^2, TODO: calibrate
static const double settle_time_s = 0.2; // s, TODO: calibrate
static const double position_tolerance_mm = 5.0; // mm, TODO: calibrate
static const double heading_tolerance_rad = 0.035; // rad, TODO: calibrate
static const double telemetry_period_s = 0.10;  // s
static const double autonomous_timeout_s = 15.0; // s, TODO: verify per routine
static const double inertial_calibration_timeout_s = 5.0; // s, TODO: verify sensor startup time
static const double inertial_calibration_period_s = 0.02; // s, VEX calibration loop period
}

#endif
