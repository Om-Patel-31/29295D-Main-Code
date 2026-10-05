#include "robot.h"
#include "variables.h"
#include "vex.h"
#include <cmath>

using namespace	vex;

competition		Competition;

void	pre_auton(void)
{
	initialize_robot(); // calibrates the IMU and zeroes the tracking wheels
}

void	autonomous(void)
{
}

// Driver response curve: deadzone, then a power curve that keeps the sign.
// Input and output are both -100..100.
float	cubicDrive(float input)
{
	float	normalized;
	float	sign;

	normalized = input / 100.0f;
	if (std::fabs(normalized) < (driver_tuning.deadzone / 100.0f))
		return (0.0f);
	sign = (normalized < 0.0f) ? -1.0f : 1.0f;
	return (sign * std::pow(std::fabs(normalized),
			driver_tuning.response_exponent) * 100.0f);
}

void	usercontrol(void)
{
	float	drive;
	float	turn;
	float	left;
	float	right;

	while (true)
	{
		drive = Controller.Axis3.position();
		turn = Controller.Axis1.position();
		drive = cubicDrive(drive);
		turn = cubicDrive(turn);
		left = drive + turn;
		right = drive - turn;
		LFM.spin(vex::fwd, left, vex::pct);
		LBM.spin(vex::fwd, left, vex::pct);
		RFM.spin(vex::fwd, right, vex::pct);
		RBM.spin(vex::fwd, right, vex::pct);
		vex::wait(10, vex::msec);
	}
}

int	main(void)
{
	Competition.autonomous(autonomous);
	Competition.drivercontrol(usercontrol);

	pre_auton();

	// Keep the program alive so the competition callbacks keep running.
	while (true)
	{
		vex::wait(100, vex::msec);
	}
}