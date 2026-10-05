#include "controls.h"
#include <cmath>

// ============================================================
// POSE ACCESS
// ============================================================

Pose get_pose()
{
	updateOdometry();
	return robot_pose;
}


float get_x()
{
	updateOdometry();
	return robot_pose.x;
}


float get_y()
{
	updateOdometry();
	return robot_pose.y;
}


float get_heading()
{
	updateOdometry();
	return robot_pose.theta;
}


// Heading-hold gains used while driving straight.
static constexpr float heading_hold_kp = 2.0f;   // TUNE: straight-line heading hold P gain
static constexpr float heading_hold_kd = 0.15f;   // TUNE: heading hold damping on IMU turn rate (untested)


// ============================================================
// STRAIGHT-LINE MOTION
//
// Moves the robot forward by distance inches.
// Uses trigonometric profile and either LADRC or PID.
// ============================================================

static void driveDistance(
	float distance,
	ControlMode mode = ControlMode::LADRC,
	float duration = 1.0f
)
{
	resetControlSystem();

	ProfileState profile;
	profile.reset(distance, duration);

	float start_x = robot_pose.x;
	float start_y = robot_pose.y;
	float start_theta = robot_pose.theta;

	ladDist.reset(0.0f);
	pidDist.reset();

	while (!profile.finished)
	{
		updateOdometry();
		profile.update(dt);

		float dx = robot_pose.x - start_x;
		float dy = robot_pose.y - start_y;

		float c = std::cos(start_theta);
		float s = std::sin(start_theta);

		float actual_distance = dx * c + dy * s;

		float velocity = odometry.v;

		float command = updateLinearController(
			mode,
			actual_distance,
			velocity,
			profile.s,
			profile.v,
			profile.a
		);

		// Heading hold: IMU-fused heading (P) with turn-rate damping (D).
		float heading_error = wrapAngle(start_theta - robot_pose.theta);
		float turn_command =
			heading_hold_kp * heading_error -
			heading_hold_kd * odometry.omega;

		DriveOutput output = mixDrive(command, turn_command);

		setDriveCommand(output.left, output.right);

		vex::task::sleep(static_cast<int>(dt * 1000.0f));
	}

	stopDrive();
}


// ============================================================
// TURN-IN-PLACE MOTION
//
// Angle is in radians.
// ============================================================

static void turnAngle(
	float angle,
	ControlMode mode = ControlMode::LADRC,
	float duration = 1.0f
)
{
	resetControlSystem();

	float start_theta = robot_pose.theta;

	float target_theta = wrapAngle(start_theta + angle);

	ProfileState profile;
	profile.reset(angle, duration);

	while (!profile.finished)
	{
		updateOdometry();
		profile.update(dt);

		float current_delta = wrapAngle(robot_pose.theta - start_theta);
		float target_delta = profile.s;
		float target_rate = profile.v;
		float target_acceleration = profile.a;

		float command = updateHeadingController(
			mode,
			current_delta,
			odometry.omega,
			target_delta,
			target_rate,
			target_acceleration
		);

		DriveOutput output = mixDrive(0.0f, command);

		setDriveCommand(output.left, output.right);

		vex::task::sleep(static_cast<int>(dt * 1000.0f));
	}

	// Heading is measured by the IMU, so it is not snapped to the target.
	(void)target_theta;

	stopDrive();
}


// ============================================================
// CUBIC BEZIER CURVE FOLLOWING
//
// The curve is defined by four control points:
//
//   p0 (start), p1 (control 1), p2 (control 2), p3 (end)
// ============================================================

static void followBezier(
	Vec2 p0,
	Vec2 p1,
	Vec2 p2,
	Vec2 p3,
	ControlMode mode = ControlMode::LADRC,
	float duration = 2.0f
)
{
	resetControlSystem();

	configureBezier(p0, p1, p2, p3);

	ProfileState profile;
	profile.reset(bezier.L, duration);

	while (!profile.finished)
	{
		updateOdometry();
		profile.update(dt);

		float path_t = arcLengthToParameter(bezier, profile.s);

		PathErrors errors = calculatePathErrors(bezier, robot_pose, path_t);

		// Feedforward from Bezier curvature.
		float curvature = bezierCurvature(bezier, path_t);
		float angular_velocity_ff = profile.v * curvature;

		// Linear tracking.
		float linear_command = updateLinearController(
			mode,
			errors.e_along,
			odometry.v,
			profile.s,
			profile.v,
			profile.a
		);

		// Cross-track correction.
		float cross_correction = errors.k_c * errors.e_cross;

		// Heading correction.
		float heading_command = updateHeadingController(
			mode,
			errors.e_theta,
			odometry.omega,
			0.0f,
			angular_velocity_ff,
			0.0f
		);

		float angular_command = heading_command + cross_correction;

		DriveOutput output = mixDrive(linear_command, angular_command);

		setDriveCommand(output.left, output.right);

		vex::task::sleep(static_cast<int>(dt * 1000.0f));
	}

	stopDrive();
}


// ============================================================
// PUBLIC MOTION CONTROL FUNCTIONS
// ============================================================

void move_forward(float distance, float duration)
{
	ensureControlInitialized();
	driveDistance(distance, ControlMode::LADRC, duration);
}


void move_forward_pid(float distance, float duration)
{
	ensureControlInitialized();
	driveDistance(distance, ControlMode::PID, duration);
}


void turn_radians(float angle, float duration)
{
	ensureControlInitialized();
	turnAngle(angle, ControlMode::LADRC, duration);
}


void turn_degrees(float angle, float duration)
{
	ensureControlInitialized();
	turnAngle(angle * pi / 180.0f, ControlMode::LADRC, duration);
}


void follow_bezier(
	float x0,
	float y0,
	float x1,
	float y1,
	float x2,
	float y2,
	float x3,
	float y3,
	float duration
)
{
	ensureControlInitialized();

	followBezier(
		{x0, y0},
		{x1, y1},
		{x2, y2},
		{x3, y3},
		ControlMode::LADRC,
		duration
	);
}


// ============================================================
// DRIVER CONTROL
//
// Cubic response with deadzone and acceleration limiting.
// ============================================================

static float driver_axis(float input)
{
	float x = input;

	float magnitude = std::fabs(x);

	if (magnitude < driver_tuning.deadzone)
	{
		return 0.0f;
	}

	float normalized = (magnitude - driver_tuning.deadzone) /
		(100.0f - driver_tuning.deadzone);

	normalized = clampf(normalized, 0.0f, 1.0f);

	float output = std::pow(normalized, driver_tuning.response_exponent);

	return sgnf(x) * output * 100.0f;
}


static float driver_acceleration(float target, float current, float gain)
{
	float error = target - current;
	float step = gain * dt * 100.0f;

	if (error > step)
		return current + step;

	if (error < -step)
		return current - step;

	return target;
}


void driver_drive(float forward, float turn)
{
	float forward_cmd = driver_axis(forward);
	float turn_cmd = driver_axis(turn);

	float target_left = forward_cmd - turn_cmd * driver_tuning.turn_gain / 10.0f;
	float target_right = forward_cmd + turn_cmd * driver_tuning.turn_gain / 10.0f;

	driver_left = driver_acceleration(target_left, driver_left, driver_tuning.acceleration_gain);
	driver_right = driver_acceleration(target_right, driver_right, driver_tuning.acceleration_gain);

	driver_left = clampf(driver_left, -100.0f, 100.0f);
	driver_right = clampf(driver_right, -100.0f, 100.0f);

	LFM.spin(vex::forward, driver_left, vex::pct);
	LBM.spin(vex::forward, driver_left, vex::pct);
	RFM.spin(vex::forward, driver_right, vex::pct);
	RBM.spin(vex::forward, driver_right, vex::pct);
}


void reset_driver()
{
	driver_left = 0.0f;
	driver_right = 0.0f;

	stopDrive();
}


// ============================================================
// SYSTEM INITIALIZATION
// ============================================================

void initialize_motion_control()
{
	if (!sensors_calibrated)
	{
		calibrateSensors();
	}

	initializeControlSystem();
	control_initialized = true;
}