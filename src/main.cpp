#include "vex.h"     // Include VEX Robotics V5 API
#include "robot.h"   // Include physical robot hardware definitions and sensor objects
#include "tuning.h"  // Include consolidated robot tuning parameters
#include "autons.h"  // Include autonomous routines and selector state
#include "quintic.h" // Include quintic profile and PID controller classes
#include "drivetrain.h" // Include shared chassis output layer (quintic arcade)
#include "ladrc2.h"  // Include LADRC active disturbance rejection controller class

using namespace vex; // Expose VEX SDK namespace

// ============================================================================
// Global System & Control Class Instantiations
// ============================================================================

// Global VEX Competition manager instance managing field state transitions
competition Competition;                                           // Field competition manager

// ============================================================================
// Auton Selection (TUNABLE - Line 20)
// ============================================================================
// Set this to choose which autonomous routine runs on field:
//   0 = Driver Control only (no auton)
//   1 = Left Side  (LS)
//   2 = Right Side (RS)  <-- default
//   3 = Skills
static const int default_auton_selection = 2;              // <-- CHANGE THIS

// Cascade lift PID controller instantiated with tuning gains
PID liftPID(tuning.lift_kp, tuning.lift_ki, tuning.lift_kd, tuning.lift_integral_max); // Cascade lift PID instance

// Drivetrain LADRC controller instance for disturbance rejection tracking
Ladrc2 dADRC(tuning.drive_wc(), tuning.drive_b0_mm(), tuning.drive_wo(), tuning.drive_max_disturb_mm()); // Drive LADRC instance

// Static flag indicating whether the pre-auton mode selection has been permanently locked
static bool s_mode_locked = false;                                 // Lock flag to permanently disable A/B selector encodings

// ----------------------------------------------------------------------------
// Pre-Autonomous Initialization
// ----------------------------------------------------------------------------
// Calibrates the inertial sensor, zeroes odometry and tracking wheels,
// then reads `default_auton_selection` (line 20) to set the auton routine:
//   0 = Driver Control only (no auton)
//   1 = Left Side  (LS)
//   2 = Right Side (RS)  <-- default
//   3 = Skills
// No controller button interaction required.
// ----------------------------------------------------------------------------
void pre_auton()
{
    imu.calibrate();                                               // Begin 3-axis inertial sensor calibration
    timer calib_timer;                                             // Start timer to detect calibration timeout
    while (imu.isCalibrating()) {                                  // Wait while inertial sensor performs background calibration
        if (calib_timer.time(sec) > tuning.imu_calib_timeout_s) {  // Check if calibration exceeded timeout threshold
            g_imu_calibration_failed = true;                       // Flag calibration failure
            break;                                                 // Abort waiting loop
        }
        wait(20, msec);                                            // Yield thread for 20ms
    }

    reset_odometry_and_sensors();                                  // Zero tracking wheels, encoders, and odometry pose
    chassis.CheckMotors();                                         // Warn on Brain screen if any drive motor is unplugged

    // Set autonomous routine based on compile-time tunable at line 20
    switch (default_auton_selection) {
        case 1:
            g_selected_auton = AutonSelection::LeftSide;
            Controller.Screen.print("Auton: Left Side   ");
            break;
        case 2:
            g_selected_auton = AutonSelection::RightSide;
            Controller.Screen.print("Auton: Right Side  ");
            break;
        case 3:
            g_selected_auton = AutonSelection::Skills;
            Controller.Screen.print("Auton: Skills      ");
            break;
        default:
            g_selected_auton = AutonSelection::RightSide;          // Fallback to Right Side
            Controller.Screen.print("Auton: Driver Only ");
            break;
    }

    s_mode_locked = true;
    Controller.rumble("-");                                        // Single rumble to confirm ready
}


// ----------------------------------------------------------------------------
// Driver Control Function (User Control)
// ----------------------------------------------------------------------------
// Executes live manual driving using quintic curve joystick mapping,
// cascade lift control, and claw clamp toggling.
// ----------------------------------------------------------------------------
void usercontrol()
{
    while (true) {                                                 // Continuous driver control loop
        const double fwd_raw = Controller.Axis3.position(percent); // Read raw throttle input from left stick Axis 3
        const double turn_raw = Controller.Axis1.position(percent);// Read raw steering input from right stick Axis 1

        chassis.Arcade(fwd_raw, turn_raw);                         // Quintic-shaped arcade drive on both sides

        if (Controller.ButtonR1.pressing()) {                      // Button R1 pressed: raise cascade lift
            lift_left.spin(forward, tuning.max_voltage, volt);     // Spin left lift motor upward at max voltage
            lift_right.spin(forward, tuning.max_voltage, volt);    // Spin right lift motor upward at max voltage
            liftPID.Reset();                                       // Reset lift PID accumulator
        } else if (Controller.ButtonR2.pressing()) {               // Button R2 pressed: lower cascade lift
            lift_left.spin(reverse, tuning.max_voltage, volt);     // Spin left lift motor downward at max voltage
            lift_right.spin(reverse, tuning.max_voltage, volt);    // Spin right lift motor downward at max voltage
            liftPID.Reset();                                       // Reset lift PID accumulator
        } else {                                                   // Neither lift button pressed: hold position
            lift_left.stop(hold);                                  // Hold left lift motor position with active brake
            lift_right.stop(hold);                                 // Hold right lift motor position with active brake
        }

        if (Controller.ButtonL1.pressing()) {                      // Button L1 pressed: clamp claw
            claw.spin(forward, tuning.claw_close_power_pct, percent); // Power claw motor closed
        } else if (Controller.ButtonL2.pressing()) {               // Button L2 pressed: open claw
            claw.spin(reverse, -tuning.claw_open_power_pct, percent); // Power claw motor open (negate to get positive value)
        } else {                                                   // Neither claw button pressed: brake claw
            claw.stop(brake);                                      // Brake claw motor to maintain grip
        }

        wait(20, msec);                                            // Yield execution for 20ms per driver loop cycle
    }
}

// ----------------------------------------------------------------------------
// Main Program Entry Point
// ----------------------------------------------------------------------------
int main()
{
    Competition.autonomous(autonomous);                            // Register autonomous callback with competition engine
    Competition.drivercontrol(usercontrol);                        // Register driver control callback with competition engine

    pre_auton();                                                   // Execute pre-auton sensor calibration and A/B selection

    while (true) {                                                 // Keep main thread alive
        wait(100, msec);                                           // Sleep 100ms between background heartbeats
    }

    return 0;                                                      // Program termination return code
}