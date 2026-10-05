#include "robot.h"
#include <cmath>

vex::brain Brain;
vex::controller Controller;

vex::motor LFM = vex::motor(vex::PORT11, vex::ratio6_1, true);
vex::motor LBM = vex::motor(vex::PORT12, vex::ratio6_1, true);
vex::motor RFM = vex::motor(vex::PORT20, vex::ratio6_1, false);
vex::motor RBM = vex::motor(vex::PORT19, vex::ratio6_1, false);

vex::rotation VRS = vex::rotation(vex::PORT4);
vex::rotation HRS = vex::rotation(vex::PORT5);

vex::inertial IMU = vex::inertial(vex::PORT6);

// --- Odometry Constants ---
const double WHEEL_DIAMETER = 2.75; // inches
const double TRACK_WIDTH = 12.0;    // Distance between tracking wheels in inches
const double PI = 3.141592653589793;

// Odometry state
double currentX = 0;
double currentY = 0;
double currentAngle = 0;

// Last recorded positions for delta calculation
double lastVRS = 0;
double lastHRS = 0;

void update_odometry() {
    // Get current rotations in inches
    double vrsPos = VRS.position(vex::rotationUnits::deg) * (WHEEL_DIAMETER * PI / 360.0);
    double hrsPos = HRS.position(vex::rotationUnits::deg) * (WHEEL_DIAMETER * PI / 360.0);
    
    // Calculate deltas (change in position)
    double deltaVRS = vrsPos - lastVRS;
    double deltaHRS = hrsPos - lastHRS;
    
    lastVRS = vrsPos;
    lastHRS = hrsPos;

    // Get current heading from IMU (converted to radians)
    currentAngle = IMU.rotation(vex::rotationUnits::deg) * (PI / 180.0);

    // Simplified 2-wheel Odometry
    currentX += deltaVRS * cos(currentAngle);
    currentY += deltaHRS; 
}

void move_to(double targetX, double targetY) {
    while (true) {
        update_odometry();

        // 1. Calculate distance to target
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < 1.5) break; // Target reached (1.5 inch tolerance)

        // 2. Calculate angle to target (relative to field)
        double targetAngle = std::atan2(dy, dx);
        double angleError = targetAngle - currentAngle;

        // Normalize angle error to [-PI, PI]
        while (angleError > PI) angleError -= 2 * PI;
        while (angleError < -PI) angleError += 2 * PI;

        // 3. Drive Logic
        if (std::abs(angleError) > 0.1) {
            // Turn towards target
            double turnSpeed = 50.0 * angleError; // Simple P-loop
            // Use explicit casting to directionType if available or simple spin(dir)
            // Since we are having trouble with the Enum, we will use the most basic spin(dir) 
            // and then set voltage separately if needed, but the SDK allows spin(dir, voltage, units)
            // Let's try the most compatible VEX way:
            LFM.spin(vex::forward, turnSpeed, vex::voltageUnits::volt);
            LBM.spin(vex::forward, turnSpeed, vex::voltageUnits::volt);
            RFM.spin(vex::reverse, turnSpeed, vex::voltageUnits::volt);
            RBM.spin(vex::reverse, turnSpeed, vex::voltageUnits::volt);
        } else {
            // Drive forward
            LFM.spin(vex::forward, 60, vex::voltageUnits::volt);
            LBM.spin(vex::forward, 60, vex::voltageUnits::volt);
            RFM.spin(vex::forward, 60, vex::voltageUnits::volt);
            RBM.spin(vex::forward, 60, vex::voltageUnits::volt);
        }
        
        vex::task::sleep(20);
    }
    
    LFM.stop(); LBM.stop(); RFM.stop(); RBM.stop();
}
