#include "robot.h"  // Include robot hardware declarations
#include "tuning.h" // Include tuning parameters for port assignments and dimensions

// ============================================================================
// Physical Robot Hardware Configuration
// ============================================================================
// Instantiates all physical motors, rotation sensors, inertial sensors, and the
// V5 Brain. This file contains ONLY the physical robot hardware definitions.
// ============================================================================

// V5 Brain instance managing display, battery, and system hardware
vex::brain Brain;

// Primary V5 wireless handheld controller
vex::controller Controller = vex::controller(vex::primary);

// -----------------------------------------------------------------------
// NOTE: Motor ports and directions are hardcoded here (not read from
// `tuning`) to avoid the C++ static-initialization-order fiasco.
// `tuning` (defined in tuning.cpp) may not be initialized yet when
// robot.cpp's globals run. Keep these in sync with tuning.cpp manually.
//
// ALWAYS use vex::PORTn. VEX device constructors take a ZERO-based index
// (vex::PORT1 == 0), so a bare `1` would silently mean port 2.
// Ports:  LF=2  LR=12  RF=1  RR=11  LiftL=3  LiftR=4  Claw=5
//         TrackPar=14  TrackPerp=17  Vision=21  IMU=20
// -----------------------------------------------------------------------

// Front-left drive motor  (port 2, reversed, 600 RPM blue)
vex::motor left_front(vex::PORT2, vex::gearSetting::ratio6_1, true);

// Rear-left drive motor   (port 12, reversed, 600 RPM blue)
vex::motor left_rear(vex::PORT12, vex::gearSetting::ratio6_1, true);

// Front-right drive motor (port 1, normal,   600 RPM blue)
vex::motor right_front(vex::PORT1, vex::gearSetting::ratio6_1, false);

// Rear-right drive motor  (port 11, normal,   600 RPM blue)
vex::motor right_rear(vex::PORT11, vex::gearSetting::ratio6_1, false);

// Left cascade lift motor  (port 3, normal,   600 RPM blue)
vex::motor lift_left(vex::PORT3, vex::gearSetting::ratio6_1, false);

// Right cascade lift motor (port 4, reversed, 600 RPM blue)
vex::motor lift_right(vex::PORT4, vex::gearSetting::ratio6_1, true);

// Claw motor (port 5, normal, 200 RPM green)
vex::motor claw(vex::PORT5, vex::gearSetting::ratio18_1, false);

// Parallel forward tracking wheel rotation sensor (port 14)
vex::rotation tracking_parallel(vex::PORT14);

// Perpendicular lateral tracking wheel rotation sensor (port 17)
vex::rotation tracking_perp(vex::PORT17);

// AI Vision sensor (port 21)
vex::aivision vision_sensor(vex::PORT21);

// V5 3-Axis Inertial Sensor (port 20)
vex::inertial imu(vex::PORT20);

// Global Odometry tracking instance initialized with lateral wheel center-offset
Odometry g_odometry(tuning.horizontal_wheel_offset_mm());

// ============================================================================
// Drivetrain Motor Position Measurement Helpers
// ============================================================================

// Returns the average rotation of the two left drive motors in degrees
double average_left_degrees()
{
    // Sum left front and left rear positions and divide by 2
    return 0.5 * (left_front.position(vex::degrees) + left_rear.position(vex::degrees));
}

// Returns the average rotation of the two right drive motors in degrees
double average_right_degrees()
{
    // Sum right front and right rear positions and divide by 2
    return 0.5 * (right_front.position(vex::degrees) + right_rear.position(vex::degrees));
}

// Returns the average linear travel of the left drive wheels in inches
double average_left_inches()
{
    // Multiply average left motor degrees by linear inches per degree
    return average_left_degrees() * inches_per_motor_degree();
}

// Returns the average linear travel of the right drive wheels in inches
double average_right_inches()
{
    // Multiply average right motor degrees by linear inches per degree
    return average_right_degrees() * inches_per_motor_degree();
}

// Returns the average linear travel of the left drive wheels in millimeters
double average_left_mm()
{
    // Multiply average left motor degrees by linear millimeters per degree
    return average_left_degrees() * mm_per_motor_degree();
}

// Returns the average linear travel of the right drive wheels in millimeters
double average_right_mm()
{
    // Multiply average right motor degrees by linear millimeters per degree
    return average_right_degrees() * mm_per_motor_degree();
}
