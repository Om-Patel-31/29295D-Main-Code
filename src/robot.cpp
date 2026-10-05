#include "robot.h"
#include "tuning/tuning.h"
#include <cmath>
#include <vector>
#include <algorithm>

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

// --- Motion Profiles ---
double get_trig_velocity(double t, double max_vel) {
    // S-Curve using sine: v(t) = max_vel * (1 - cos(pi * t)) / 2
    return max_vel * (1.0 - std::cos(M_PI * t)) / 2.0;
}

void move_to(double targetX, double targetY) {
    double startTime = vex::timer::systemHighResolution();
    double duration = 2.0; // Estimated duration for trig ramp

    while (true) {
        update_odometry();
        double elapsed = (vex::timer::systemHighResolution() - startTime) / 1000000.0;
        double t = std::min(1.0, elapsed / duration);
        double current_max_vel = get_trig_velocity(t, 60.0);


        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double distance = std::sqrt(dx*dx + dy*dy);

        if (distance < 1.5) break;

        double targetAngle = std::atan2(dy, dx);
        double angleError = targetAngle - currentAngle;
        while (angleError > PI) angleError -= 2 * PI;
        while (angleError < -PI) angleError += 2 * PI;

        if (std::abs(angleError) > 0.1) {
            double turnSpeed = 50.0 * angleError;
            LFM.spin(vex::forward, turnSpeed, vex::voltageUnits::volt);
            LBM.spin(vex::forward, turnSpeed, vex::voltageUnits::volt);
            RFM.spin(vex::reverse, turnSpeed, vex::voltageUnits::volt);
            RBM.spin(vex::reverse, turnSpeed, vex::voltageUnits::volt);
        } else {
            LFM.spin(vex::forward, current_max_vel, vex::voltageUnits::volt);
            LBM.spin(vex::forward, current_max_vel, vex::voltageUnits::volt);
            RFM.spin(vex::forward, current_max_vel, vex::voltageUnits::volt);
            RBM.spin(vex::forward, current_max_vel, vex::voltageUnits::volt);
        }
        vex::task::sleep(20);
    }
    LFM.stop(); LBM.stop(); RFM.stop(); RBM.stop();
}

