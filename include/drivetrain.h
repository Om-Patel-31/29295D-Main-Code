#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "vex.h"     // Include VEX Robotics V5 API
#include "quintic.h" // Include QuinticCurve joystick shaper

// ============================================================================
// Tank Drivetrain Output Layer
// ============================================================================
// The ONLY place that commands the four chassis motors. Driver control and
// every autonomous motion (drive, turn, bdrive, sdrive) route through here so
// the left and right sides are always driven by the same code path.
//
// Sign convention (both sides):
//   +volts = wheel drives the robot forward
//   left > right  = robot turns right (clockwise)
// Motor direction reversal is handled once in robot.cpp, never here.
// ============================================================================

class Drivetrain
{
public:
    // Constructor setting joystick deadband for the quintic driver curves
    explicit Drivetrain(double deadband_pct = 5.0);

    // Commands left and right side voltages, scaling both proportionally if either exceeds max_voltage
    void SetVoltage(double left_volts, double right_volts);

    // Stops all four drive motors with the given brake mode
    void Stop(vex::brakeType mode = vex::brake);

    // Quintic-shaped arcade driver control from raw joystick percentages [-100, 100]
    void Arcade(double fwd_pct, double turn_pct);

    // Checks all four drive motors are plugged in; prints any missing motor to the Brain screen
    bool CheckMotors();

    // Last voltage commanded to the left side (Volts)
    double left_volts() const { return left_volts_; }

    // Last voltage commanded to the right side (Volts)
    double right_volts() const { return right_volts_; }

private:
    QuinticCurve throttle_curve_; // Forward/reverse joystick shaper (Axis 3)
    QuinticCurve turn_curve_;     // Steering joystick shaper (Axis 1)
    double left_volts_;           // Last commanded left side voltage
    double right_volts_;          // Last commanded right side voltage
};

// Global drivetrain instance shared by driver control and autonomous
extern Drivetrain chassis;

#endif // DRIVETRAIN_H
