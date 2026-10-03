#include "headers/robot.h"

DriverTuning driver_tuning(
	7.0f,	// deadzone
	3.0f,	// response exponent
	10.0f,	// acceleration gain
	7.0f	// turn gain
);

void initialize_robot() 
{
	LFM.setStopping(vex::brake);
	LBM.setStopping(vex::brake);
	RFM.setStopping(vex::brake);
	RBM.setStopping(vex::brake);
	
	LFM.resetPosition();
	LBM.resetPosition();
	RFM.resetPosition();
	RBM.resetPosition();
}