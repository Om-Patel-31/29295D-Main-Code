#include "drivetrain.h" // Include Drivetrain declaration
#include "robot.h"      // Include chassis motor references
#include "tuning.h"     // Include voltage limits and driver gains
#include <algorithm>    // Include std::max
#include <cmath>        // Include std::fabs, std::isfinite

// Global drivetrain instance with a 5% joystick deadband
Drivetrain chassis(5.0);

// Constructor: throttle curve 25% linear / 75% quintic, turn curve 35% linear / 65% quintic
Drivetrain::Drivetrain(double deadband_pct)
    : throttle_curve_(deadband_pct, 0.25, 0.75),                   // Throttle shaper
      turn_curve_(deadband_pct, 0.35, 0.65),                       // Steering shaper
      left_volts_(0.0),                                            // No left command yet
      right_volts_(0.0)                                            // No right command yet
{
}

// Commands both sides, preserving the left:right ratio when saturated
void Drivetrain::SetVoltage(double left_volts, double right_volts)
{
    if (!std::isfinite(left_volts) || !std::isfinite(right_volts)) { // Never send NaN/Inf to the motors
        Stop(vex::brake);                                          // Fail safe: brake the chassis
        return;
    }

    const double limit = tuning.max_voltage;                       // Maximum allowed motor voltage
    const double peak = std::max(std::fabs(left_volts), std::fabs(right_volts)); // Largest requested magnitude
    if (peak > limit) {                                            // Either side exceeds the limit
        const double scale = limit / peak;                         // Common scale keeps turn arc unchanged
        left_volts *= scale;                                       // Scale left side
        right_volts *= scale;                                      // Scale right side
    }

    left_front.spin(vex::forward, left_volts, vex::volt);          // Front-left
    left_rear.spin(vex::forward, left_volts, vex::volt);           // Rear-left
    right_front.spin(vex::forward, right_volts, vex::volt);        // Front-right
    right_rear.spin(vex::forward, right_volts, vex::volt);         // Rear-right

    left_volts_ = left_volts;                                      // Record left command
    right_volts_ = right_volts;                                    // Record right command
}

// Stops all four drive motors
void Drivetrain::Stop(vex::brakeType mode)
{
    left_front.stop(mode);                                         // Front-left
    left_rear.stop(mode);                                          // Rear-left
    right_front.stop(mode);                                        // Front-right
    right_rear.stop(mode);                                         // Rear-right
    left_volts_ = 0.0;                                             // Clear left command
    right_volts_ = 0.0;                                            // Clear right command
}

// Quintic-shaped arcade: Axis 3 throttle, Axis 1 steering
void Drivetrain::Arcade(double fwd_pct, double turn_pct)
{
    const double fwd = throttle_curve_.Calculate(fwd_pct);         // Shaped throttle percentage
    const double turn = turn_curve_.Calculate(turn_pct);           // Shaped steering percentage

    if (fwd == 0.0 && turn == 0.0) {                               // Both sticks inside deadband
        Stop(vex::brake);                                          // Hold the chassis instead of coasting
        return;
    }

    const double v_fwd = fwd * tuning.driver_accel_gain;           // Throttle in Volts
    const double v_turn = turn * tuning.driver_turn_gain;          // Steering in Volts
    SetVoltage(v_fwd + v_turn, v_fwd - v_turn);                    // Right stick right -> left faster -> turn right
}

// Verifies every drive motor is detected on its smart port
bool Drivetrain::CheckMotors()
{
    struct Entry { vex::motor& motor; const char* name; };         // Motor reference with display name
    Entry entries[] = {
        {left_front, "Left Front"},
        {left_rear, "Left Rear"},
        {right_front, "Right Front"},
        {right_rear, "Right Rear"},
    };

    bool all_ok = true;                                            // Assume every motor is present
    int32_t row = 20;                                              // Brain screen text row (pixels)
    for (Entry& e : entries) {
        if (!e.motor.installed()) {                                // Motor not detected on its port
            if (all_ok) {                                          // First failure: set warning color
                Brain.Screen.setPenColor(vex::color(255, 100, 100));
            }
            Brain.Screen.printAt(20, row, true, "DRIVE MOTOR MISSING: %s (port %d)",
                                 e.name, static_cast<int>(e.motor.index()) + 1); // index() is zero-based
            row += 20;                                             // Next line
            all_ok = false;                                        // Record failure
        }
    }
    if (!all_ok) Controller.rumble("---");                         // Long rumble alerts the driver
    return all_ok;
}
