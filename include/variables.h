#ifndef VARIABLES_H
#define VARIABLES_H

#include "robot.h"
#include <cmath>

// ============================================================
// FIXED CONSTANTS
// ============================================================

constexpr float pi = 3.14159265358979323846f;
constexpr float dt = 0.010f;              // 10 ms control period
constexpr float v_cmd = 12.0f;            // normalized controller output
constexpr float eps = 1.0e-6f;

// TUNE: assumes 3.25in wheel and 1:1 gearing; include any gear ratio
constexpr float inches_per_motor_degree =
	(3.25f * pi) / 360.0f;


// ============================================================
// CALCULATED VARIABLES
// ============================================================

// Basic geometry structure
struct Vec2
{
	float x;
	float y;
};

// Pose structure
struct Pose
{
	float x = 0.0f;
	float y = 0.0f;
	float theta = 0.0f;
};

// Geometry configuration
//
// Robot frame: +X = forward, +Y = left, +theta = counter-clockwise.
//
// VRS (vertical rotation sensor)   : tracking wheel facing the same way
//                                    as the robot; reads forward travel.
// HRS (horizontal rotation sensor) : tracking wheel mounted 90 degrees
//                                    from VRS; reads sideways (left) travel.
// IMU (inertial sensor)            : heading / turn rate.
struct Geometry
{
	float track_w = 16.0f;   // TUNE: measure wheel-to-wheel track width (in); affects encoder heading fallback
	float wheel_diameter = 3.25f;   // TUNE: drive wheel diameter (in); keep in sync with inches_per_motor_degree

	// Tracking wheel diameters (inches)
	float d_v = 2.0f;   // VRS wheel  TUNE: set to your tracking wheel diameter (in)
	float d_h = 2.0f;   // HRS wheel  TUNE: set to your tracking wheel diameter (in)

	// Tracking wheel offsets from the tracking centre (inches)
	//   s_v : LATERAL offset of the VRS wheel   (+ = left of centre)
	//   s_h : FORWARD offset of the HRS wheel   (+ = ahead of centre)
	float s_v = 0.0f;   // TUNE: measure VRS lateral offset (in, + = left). Refine by spinning in place; position should not drift
	float s_h = 0.0f;   // TUNE: measure HRS forward offset (in, + = ahead). Refine by spinning in place; position should not drift

	Vec2 imu{0.0f, 0.0f};
	Vec2 ctr{0.0f, 0.0f};

	Vec2 r_imu() const
	{
		return {imu.x - ctr.x, imu.y - ctr.y};
	}
};

// Odometry state
struct Odometry
{
	// ---- Tracking wheel deltas for the latest update (inches) ----
	float dv_raw = 0.0f;     // VRS, as measured
	float dh_raw = 0.0f;     // HRS, as measured

	float dv_corr = 0.0f;    // VRS, rotation-corrected (centre travel, forward)
	float dh_corr = 0.0f;    // HRS, rotation-corrected (centre travel, left)

	// ---- Heading ----
	float dtheta = 0.0f;       // fused heading change used for this update
	float dtheta_imu = 0.0f;   // from IMU
	float dtheta_enc = 0.0f;   // from drive-motor encoders (differential)
	float theta_m = 0.0f;      // mid-step heading

	float heading_imu = 0.0f;  // accumulated since reset (diagnostic)
	float heading_enc = 0.0f;  // accumulated since reset (diagnostic)

	// ---- Local displacement in the world frame for the latest update ----
	float dx_1 = 0.0f;
	float dy_1 = 0.0f;

	float ds_drive = 0.0f;     // forward travel from drive encoders
	float ds_fused = 0.0f;     // fused forward travel (VRS + drive encoders)

	// ---- Filtered rates ----
	float alpha_f = 0.3f;   // TUNE: v / omega low-pass (0..1). Higher = faster but noisier

	float v = 0.0f;           // forward velocity (in/s)
	float omega = 0.0f;       // turn rate (rad/s, CCW +)

	// ---- Sensor health (updated every call) ----
	bool imu_ok = false;
	bool vrs_ok = false;
	bool hrs_ok = false;
	bool slip_detected = false;   // drive wheels disagree with VRS

	// ---- Baselines from the previous call ----
	float previous_left = 0.0f;
	float previous_right = 0.0f;
	float previous_vrs = 0.0f;
	float previous_hrs = 0.0f;
	float previous_imu = 0.0f;

	bool previous_imu_ok = false;
	bool previous_vrs_ok = false;
	bool previous_hrs_ok = false;

	// ---- Rate accumulation (so repeated get_pose() calls within one
	//      control tick do not corrupt velocity estimates) ----
	float acc_ds = 0.0f;
	float acc_dtheta = 0.0f;
	float acc_time = 0.0f;
	unsigned long long previous_time_us = 0;

	bool initialized = false;
};

// Sensor fusion tuning
struct Fusion
{
	// Weight on VRS for forward travel. The remainder comes from the drive
	// encoders. VRS is unpowered and does not slip, so it dominates.
	float w_vrs = 0.90f;   // TUNE: 0..1 weight on VRS vs drive encoders

	// If drive encoders and VRS disagree by more than this in one update
	// (inches), the drive wheels are assumed to be slipping and VRS is
	// trusted completely for that update.
	float slip_gate = 0.35f;   // TUNE: inches per update; lower = more sensitive slip detection

	// Minimum time (seconds) accumulated before a velocity is computed.
	float min_rate_window = 0.005f;   // TUNE: seconds; larger = smoother v/omega, more lag
};

// Global variable declarations
extern Geometry geometry;
extern Pose robot_pose;
extern Odometry odometry;
extern Fusion fusion;

// Sensors VRS, HRS, IMU are declared in robot.h and defined in robot.cpp

extern bool sensors_calibrated;


// ============================================================
// TUNED VARIABLES
// ============================================================

// Driver tuning structure
struct DriverTuning
{
	float deadzone;
	float response_exponent;
	float acceleration_gain;
	float turn_gain;

	DriverTuning(float dz, float re, float ag, float tg)
		: deadzone(dz), response_exponent(re),
		  acceleration_gain(ag), turn_gain(tg) {}
};

extern DriverTuning driver_tuning;

// Motion limits
struct Limits
{
	float V_max = 50.0f;   // TUNE: max linear velocity (in/s)
	float A_max = 120.0f;   // TUNE: max linear acceleration (in/s^2)

	float w_max = 4.0f;   // TUNE: max turn rate (rad/s)
	float alpha_max = 12.0f;   // TUNE: max angular acceleration (rad/s^2)

	float V_wheel = 70.0f;   // TUNE: max wheel speed (in/s)
};

extern Limits limits;

// Plant model for linear and turning control
struct Plant
{
	float k_S = 0.0f;
	float k_V = 0.0f;
	float k_A = 0.0f;

	float b0() const
	{
		if (std::fabs(k_A) < eps)
			return 1.0f;
		return 1.0f / k_A;
	}
};

extern Plant linearPlant;
extern Plant turnPlant;

// Driver state variables
extern float driver_left;
extern float driver_right;

// Control system initialization flag
extern bool control_initialized;


// ============================================================
// HELPER FUNCTIONS
// ============================================================

float clampf(float x, float lo, float hi);
float sgnf(float x);
float wrapAngle(float angle);
float lerpf(float a, float b, float t);

// Vec2 operators
Vec2 operator+(const Vec2& a, const Vec2& b);
Vec2 operator-(const Vec2& a, const Vec2& b);
Vec2 operator*(const Vec2& a, float s);
Vec2 operator/(const Vec2& a, float s);

float dot(const Vec2& a, const Vec2& b);
float cross(const Vec2& a, const Vec2& b);
float magnitude(const Vec2& a);
Vec2 normalize(const Vec2& a);

// Odometry and pose access
void updateOdometry();
void resetOdometry();
float getLeftDistance();
float getRightDistance();

// Drive motor output (left/right in the +/-v_cmd range; stopDrive brakes all four motors)
void setDriveCommand(float left, float right);
void stopDrive();

// Tracking wheel / inertial access (inches, radians CCW+)
float getVerticalDistance();     // VRS
float getHorizontalDistance();   // HRS
float getImuHeading();           // IMU, unbounded, CCW positive

bool vrsHealthy();
bool hrsHealthy();
bool imuHealthy();

// Calibrate IMU and zero rotation sensors (blocks until IMU is ready)
void calibrateSensors();

#endif // VARIABLES_H