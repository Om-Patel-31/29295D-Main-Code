#include "variables.h"
#include <cmath>

// ============================================================
// GLOBAL VARIABLE DEFINITIONS
// ============================================================

// Driver tuning configuration
DriverTuning driver_tuning(
	7.0f,	// deadzone  TUNE: joystick % ignored near centre
	3.0f,	// response exponent  TUNE: 1 = linear, higher = finer low-speed control
	10.0f,	// acceleration gain  TUNE: higher = snappier, lower = smoother ramp
	7.0f	// turn gain  TUNE: turn sensitivity (10 = 1.0x)
);

// Motion limits
Limits limits;

// Geometry configuration
Geometry geometry;

// Pose tracking
Pose robot_pose;

// Odometry state
Odometry odometry;

// Sensor fusion tuning
Fusion fusion;

// ============================================================
// SENSORS
//
// VRS, HRS and IMU are declared in robot.h and defined in robot.cpp
// (ports and reverse flags are set there).
//
// VRS : must read positive when the robot drives FORWARD.
// HRS : must read positive when the robot moves LEFT.
//       (add the reverse flag in robot.cpp if your wheel reads the other way)
// IMU : getImuHeading() converts VEX clockwise-positive rotation() to
//       counter-clockwise-positive.
// ============================================================

bool sensors_calibrated = false;

// Plant models (initialized with tuning values)
Plant linearPlant = []()
{
	Plant p;
	p.k_S = 0.9f;   // TUNE: linear static friction (volts)
	p.k_V = 0.17f;   // TUNE: linear velocity gain (volts per in/s)
	p.k_A = 0.020f;   // TUNE: linear acceleration gain (volts per in/s^2)
	return p;
}();

Plant turnPlant = []()
{
	Plant p;
	p.k_S = 1.0f;   // TUNE: turn static friction (volts)
	p.k_V = 1.90f;   // TUNE: turn velocity gain (volts per rad/s)
	p.k_A = 0.200f;   // TUNE: turn acceleration gain (volts per rad/s^2)
	return p;
}();

// Driver state
float driver_left = 0.0f;
float driver_right = 0.0f;

// Control system initialization flag
bool control_initialized = false;


// ============================================================
// BASIC HELPER FUNCTIONS
// ============================================================

float clampf(float x, float lo, float hi)
{
	if (x < lo)
		return lo;
	if (x > hi)
		return hi;
	return x;
}


float sgnf(float x)
{
	if (x > 0.0f)
		return 1.0f;
	if (x < 0.0f)
		return -1.0f;
	return 0.0f;
}


float wrapAngle(float angle)
{
	while (angle > pi)
		angle -= 2.0f * pi;
	while (angle < -pi)
		angle += 2.0f * pi;
	return angle;
}


float lerpf(float a, float b, float t)
{
	return a + (b - a) * t;
}


// ============================================================
// VEC2 OPERATORS
// ============================================================

Vec2 operator+(const Vec2& a, const Vec2& b)
{
	return {a.x + b.x, a.y + b.y};
}


Vec2 operator-(const Vec2& a, const Vec2& b)
{
	return {a.x - b.x, a.y - b.y};
}


Vec2 operator*(const Vec2& a, float s)
{
	return {a.x * s, a.y * s};
}


Vec2 operator/(const Vec2& a, float s)
{
	if (std::fabs(s) < eps)
		return {0.0f, 0.0f};
	return {a.x / s, a.y / s};
}


float dot(const Vec2& a, const Vec2& b)
{
	return a.x * b.x + a.y * b.y;
}


float cross(const Vec2& a, const Vec2& b)
{
	return a.x * b.y - a.y * b.x;
}


float magnitude(const Vec2& a)
{
	return std::sqrt(a.x * a.x + a.y * a.y);
}


Vec2 normalize(const Vec2& a)
{
	float m = magnitude(a);
	if (m < eps)
		return {0.0f, 0.0f};
	return a / m;
}


// ============================================================
// ROBOT INITIALIZATION
// ============================================================

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

	calibrateSensors();
}


// ============================================================
// SENSOR CALIBRATION AND HEALTH
// ============================================================

void calibrateSensors()
{
	IMU.calibrate();
	vex::task::sleep(100);

	// Wait for the IMU, with a 5 s safety timeout.
	int waited = 0;
	while (IMU.isCalibrating() && waited < 5000)
	{
		vex::task::sleep(20);
		waited += 20;
	}

	IMU.resetRotation();
	VRS.resetPosition();
	HRS.resetPosition();

	sensors_calibrated = true;
}


bool imuHealthy()
{
	return IMU.installed() && !IMU.isCalibrating();
}


bool vrsHealthy()
{
	return VRS.installed();
}


bool hrsHealthy()
{
	return HRS.installed();
}


// ============================================================
// TRACKING WHEEL AND IMU READINGS
// ============================================================

float getVerticalDistance()
{
	float degrees = static_cast<float>(VRS.position(vex::degrees));
	return degrees * (pi * geometry.d_v / 360.0f);
}


float getHorizontalDistance()
{
	float degrees = static_cast<float>(HRS.position(vex::degrees));
	return degrees * (pi * geometry.d_h / 360.0f);
}


// VEX rotation() is clockwise-positive; pose theta is counter-clockwise-positive.
float getImuHeading()
{
	float degrees = static_cast<float>(IMU.rotation(vex::degrees));
	return -degrees * pi / 180.0f;
}


// ============================================================
// DRIVE ENCODER DISTANCE
// ============================================================

float getLeftDistance()
{
	float front = static_cast<float>(LFM.position(vex::degrees));
	float back = static_cast<float>(LBM.position(vex::degrees));
	float average_degrees = 0.5f * (front + back);
	return average_degrees * inches_per_motor_degree;
}


float getRightDistance()
{
	float front = static_cast<float>(RFM.position(vex::degrees));
	float back = static_cast<float>(RBM.position(vex::degrees));
	float average_degrees = 0.5f * (front + back);
	return average_degrees * inches_per_motor_degree;
}


// ============================================================
// ODOMETRY RESET
// ============================================================

void resetOdometry()
{
	LFM.resetPosition();
	LBM.resetPosition();
	RFM.resetPosition();
	RBM.resetPosition();

	robot_pose = Pose{};
	odometry = Odometry{};
}


// ============================================================
// ODOMETRY UPDATE  (multi-sensor fusion)
//
// Positive X = forward.
// Positive Y = left.
// Positive theta = counter-clockwise.
//
// Sensors:
//   VRS   vertical tracking wheel   -> forward travel
//   HRS   horizontal tracking wheel -> sideways travel
//   IMU   inertial sensor           -> heading change
//   Drive-motor encoders            -> redundant forward travel and
//                                      heading; fallback for any dead sensor
//
// Pipeline each update:
//   1. Heading change   : IMU, or drive encoders if the IMU is unavailable.
//   2. Tracking wheels  : remove the rotation-induced component using the
//                         wheel offsets s_v and s_h.
//   3. Forward travel   : VRS blended with drive encoders (slip-gated).
//   4. Pose integration : exact constant-twist (arc) integration using
//                         both forward AND sideways travel.
//   5. Rates            : v and omega from accumulated deltas over the
//                         real elapsed time.
// ============================================================

void updateOdometry()
{
	// ---- Read sensors ----
	bool imu_ok = imuHealthy();
	bool vrs_ok = vrsHealthy();
	bool hrs_ok = hrsHealthy();

	float left_distance = getLeftDistance();
	float right_distance = getRightDistance();

	float vrs_now = vrs_ok ? getVerticalDistance() : 0.0f;
	float hrs_now = hrs_ok ? getHorizontalDistance() : 0.0f;
	float imu_now = imu_ok ? getImuHeading() : 0.0f;

	unsigned long long now_us = vex::timer::systemHighResolution();

	// ---- First call after reset: record baselines only ----
	if (!odometry.initialized)
	{
		odometry.previous_left = left_distance;
		odometry.previous_right = right_distance;
		odometry.previous_vrs = vrs_now;
		odometry.previous_hrs = hrs_now;
		odometry.previous_imu = imu_now;

		odometry.previous_vrs_ok = vrs_ok;
		odometry.previous_hrs_ok = hrs_ok;
		odometry.previous_imu_ok = imu_ok;

		odometry.imu_ok = imu_ok;
		odometry.vrs_ok = vrs_ok;
		odometry.hrs_ok = hrs_ok;

		odometry.previous_time_us = now_us;
		odometry.initialized = true;
		return;
	}

	// ---- Deltas (a sensor that just came back online is re-baselined) ----
	float dl = left_distance - odometry.previous_left;
	float dr = right_distance - odometry.previous_right;

	float dv_raw = (vrs_ok && odometry.previous_vrs_ok) ? (vrs_now - odometry.previous_vrs) : 0.0f;
	float dh_raw = (hrs_ok && odometry.previous_hrs_ok) ? (hrs_now - odometry.previous_hrs) : 0.0f;
	bool imu_valid = imu_ok && odometry.previous_imu_ok;
	float dtheta_imu = imu_valid ? (imu_now - odometry.previous_imu) : 0.0f;

	odometry.previous_left = left_distance;
	odometry.previous_right = right_distance;
	odometry.previous_vrs = vrs_now;
	odometry.previous_hrs = hrs_now;
	odometry.previous_imu = imu_now;

	bool vrs_valid = vrs_ok && odometry.previous_vrs_ok;
	bool hrs_valid = hrs_ok && odometry.previous_hrs_ok;

	odometry.previous_vrs_ok = vrs_ok;
	odometry.previous_hrs_ok = hrs_ok;
	odometry.previous_imu_ok = imu_ok;

	odometry.imu_ok = imu_ok;
	odometry.vrs_ok = vrs_ok;
	odometry.hrs_ok = hrs_ok;

	float elapsed = static_cast<float>(now_us - odometry.previous_time_us) * 1.0e-6f;
	odometry.previous_time_us = now_us;
	elapsed = clampf(elapsed, 0.0f, 0.1f);

	// ---- 1. Heading change ----
	float ds_drive = 0.5f * (dl + dr);
	float dtheta_enc = (dr - dl) / geometry.track_w;

	float dtheta = imu_valid ? dtheta_imu : dtheta_enc;

	odometry.dtheta_imu = dtheta_imu;
	odometry.dtheta_enc = dtheta_enc;
	odometry.heading_imu += dtheta_imu;
	odometry.heading_enc += dtheta_enc;

	// ---- 2. Remove rotation-induced tracking wheel travel ----
	// A wheel at lateral offset s_v moves (-s_v * dtheta) forward during a
	// CCW turn; a wheel at forward offset s_h moves (+s_h * dtheta) left.
	float dv_corr = dv_raw + geometry.s_v * dtheta;
	float dh_corr = dh_raw - geometry.s_h * dtheta;

	// ---- 3. Forward travel fusion ----
	float ds;

	odometry.slip_detected = false;

	if (vrs_valid)
	{
		float disagreement = std::fabs(ds_drive - dv_corr);
		float w = fusion.w_vrs;

		if (disagreement > fusion.slip_gate)
		{
			w = 1.0f;
			odometry.slip_detected = true;
		}

		ds = w * dv_corr + (1.0f - w) * ds_drive;
	}
	else
	{
		ds = ds_drive;
	}

	float dh = hrs_valid ? dh_corr : 0.0f;

	// ---- 4. Pose integration ----
	float theta_old = robot_pose.theta;
	float theta_new = theta_old + dtheta;
	float theta_mid = theta_old + 0.5f * dtheta;

	// Constant body-twist (ds, dh, dtheta) integrated exactly.
	float sin_term;   // sin(dtheta) / dtheta
	float cos_term;   // (1 - cos(dtheta)) / dtheta

	if (std::fabs(dtheta) < 1.0e-5f)
	{
		sin_term = 1.0f - dtheta * dtheta / 6.0f;
		cos_term = 0.5f * dtheta;
	}
	else
	{
		sin_term = std::sin(dtheta) / dtheta;
		cos_term = (1.0f - std::cos(dtheta)) / dtheta;
	}

	float dx_local = ds * sin_term - dh * cos_term;
	float dy_local = ds * cos_term + dh * sin_term;

	float c = std::cos(theta_old);
	float s = std::sin(theta_old);

	float dx = dx_local * c - dy_local * s;
	float dy = dx_local * s + dy_local * c;

	robot_pose.x += dx;
	robot_pose.y += dy;
	robot_pose.theta = wrapAngle(theta_new);

	// ---- 5. Rates over the real elapsed time ----
	odometry.acc_ds += ds;
	odometry.acc_dtheta += dtheta;
	odometry.acc_time += elapsed;

	if (odometry.acc_time >= fusion.min_rate_window)
	{
		float v_raw = odometry.acc_ds / odometry.acc_time;
		float w_raw = odometry.acc_dtheta / odometry.acc_time;

		odometry.v = odometry.alpha_f * v_raw + (1.0f - odometry.alpha_f) * odometry.v;
		odometry.omega = odometry.alpha_f * w_raw + (1.0f - odometry.alpha_f) * odometry.omega;

		odometry.acc_ds = 0.0f;
		odometry.acc_dtheta = 0.0f;
		odometry.acc_time = 0.0f;
	}

	odometry.dv_raw = dv_raw;
	odometry.dh_raw = dh_raw;
	odometry.dv_corr = dv_corr;
	odometry.dh_corr = dh_corr;
	odometry.dtheta = dtheta;
	odometry.ds_drive = ds_drive;
	odometry.ds_fused = ds;

	odometry.dx_1 = dx;
	odometry.dy_1 = dy;
	odometry.theta_m = theta_mid;
}


// ============================================================
// MOTOR OUTPUT HELPERS
// ============================================================

void stopDrive()
{
	LFM.stop(vex::brake);
	LBM.stop(vex::brake);
	RFM.stop(vex::brake);
	RBM.stop(vex::brake);
}


void setDriveCommand(float left, float right)
{
	float left_percent =
		clampf(left / v_cmd * 100.0f, -100.0f, 100.0f);

	float right_percent =
		clampf(right / v_cmd * 100.0f, -100.0f, 100.0f);

	LFM.spin(vex::forward, left_percent, vex::pct);
	LBM.spin(vex::forward, left_percent, vex::pct);
	RFM.spin(vex::forward, right_percent, vex::pct);
	RBM.spin(vex::forward, right_percent, vex::pct);
}