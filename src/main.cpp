#include "vex.h"
#include "include/robot.h"
using namespace vex;

competition Competition;

void pre_auton(void) {

}


void autonomous(void) {
	
}

//driver functions
void cubicDrive(float input) 
{
	float normalized = input / 100.0f;
	
	if (std::fabs(normalized) < (driver_tuning.deadzone / 100.0f)) return 0.0;
	
	return normalized ** driver_tuning.response_exponent * 100.0f;
}

void usercontrol(void) {
	while (true)
	{
		float drive = Controller.Axis3.position();
		float turn = Controller.Axis2.position();
		
		drive = cubicDrive(drive);
		turn = cubicDrive(turn);
		
		float left = forward + turn;
		float right = forward - turn;
		
		LFM.spin(vex::fwd, left, vex::pct);
		LBM.spin(vex::fwd, left, vex::pct);
		RFM.spin(vex::fwd, left, vex::pct);
		RBM.spin(vex::fwd, left, vex::pct);
		
		vex::wait(0.1, vex::msec);
	}
}


int main() {
	Competition.autonomous(autonomous);
	Competition.drivercontrol(usercontrol);

	pre_auton();

}