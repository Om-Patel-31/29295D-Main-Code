/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Description:  VEX competition template entry point                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Competition entry point. This file only connects VEX competition callbacks
// to the hardware/application implementation in robot.cpp.
#include "vex.h"

using namespace vex;

// The competition object registers the autonomous and driver-control callbacks.
competition Competition;

// These functions are implemented by the robot-specific code in robot.cpp.
void pre_auton();
void autonomous();
void usercontrol();
void run_screen_selector();

int main()
{
    // Register the autonomous callback used during the competition autonomous period.
    Competition.autonomous(autonomous);
    // Register the driver-control callback used during the competition driver period.
    Competition.drivercontrol(usercontrol);
    // Run one-time sensor calibration and setup before competition tasks start.
    pre_auton();

    // Keep the main task alive while the VEX runtime runs the registered callbacks.
    run_screen_selector();
    return 0;
}