#ifndef MOTION_H
#define MOTION_H

#include "variables.h"

// ============================================================
// CONTROL MODE ENUM
// ============================================================

enum class ControlMode
{
	LADRC,
	PID
};


// ============================================================
// MOTION PROFILE STATE
// ============================================================

struct ProfileState
{
	float D = 0.0f;
	float T = 1.0f;

	float s = 0.0f;
	float v = 0.0f;
	float a = 0.0f;
	float j = 0.0f;

	float rdot = 0.0f;
	float rddot = 0.0f;

	float time = 0.0f;

	bool finished = false;

	float A_p() const;
	void reset(float distance, float duration);
	void update(float elapsed);
};


// ============================================================
// LADRC (LINEAR ACTIVE DISTURBANCE REJECTION CONTROL)
// ============================================================

struct ladrcState
{
	float z1 = 0.0f;
	float z2 = 0.0f;
	float z3 = 0.0f;

	float e = 0.0f;
	float u = 0.0f;

	float b0 = 1.0f;

	float w_o = 48.0f;	// needs tuning. figure out how.
	float w_c = 12.0f;	// needs tuning. figure out how.

	float beta1() const;
	float beta2() const;
	float beta3() const;
	float k_p() const;
	float k_d() const;

	void reset(float y0);
	float update(float y, float r, float rd, float rdd);
};

extern ladrcState ladDist;
extern ladrcState ladHead;


// ============================================================
// PID CONTROLLER
// ============================================================

struct pidState
{
	float k_P = 0.0f;
	float k_D = 0.0f;
	float k_I = 0.0f;

	float integ = 0.0f;
	float e = 0.0f;

	float k_S = 0.0f;
	float k_V = 0.0f;
	float k_A = 0.0f;

	void configure(const Plant& p, float w_c, float ki_scale);
	void reset();
	float update(float y, float velocity, float r, float rd, float rdd);
};

extern pidState pidDist;
extern pidState pidHead;


// ============================================================
// CUBIC BEZIER CURVE
// ============================================================

struct BezierState
{
	static constexpr int N = 64;

	Vec2 p0{0.0f, 0.0f};  // Start point
	Vec2 p1{0.0f, 0.0f};  // First control point
	Vec2 p2{0.0f, 0.0f};  // Second control point
	Vec2 p3{0.0f, 0.0f};  // End point

	float q = 0.0f;

	float L = 0.0f;  // Arc length

	float kappa_max = 0.0f;  // Maximum curvature

	float s_table[N + 1]{};  // Arc length lookup table
};

extern BezierState bezier;


// ============================================================
// BEZIER CURVE FUNCTIONS
// ============================================================

Vec2 bezierPosition(const BezierState& curve, float t);
Vec2 bezierDerivative(const BezierState& curve, float t);
Vec2 bezierSecondDerivative(const BezierState& curve, float t);
float bezierCurvature(const BezierState& curve, float t);

void buildBezierArcTable();
void configureBezier(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3);
float arcLengthToParameter(const BezierState& curve, float s);


// ============================================================
// PATH ERRORS
// ============================================================

struct PathErrors
{
	float e_along = 0.0f;
	float e_cross = 0.0f;

	float e_theta = 0.0f;

	float k_theta = 0.0f;
	float k_c = 2.0f;		// needs tuning. figure out how.
};

PathErrors calculatePathErrors(
	const BezierState& curve,
	const Pose& pose,
	float t
);

float projectPoseOntoCurve(
	const BezierState& curve,
	const Pose& pose
);


// ============================================================
// DIFFERENTIAL DRIVE OUTPUT
// ============================================================

struct DriveOutput
{
	float left = 0.0f;
	float right = 0.0f;
};

DriveOutput mixDrive(float linear, float angular);


// ============================================================
// CONTROLLER FUNCTIONS
// ============================================================

void applyGains(float w_c, float w_o, float ki_scale);
void resetControllers();
void resetControlSystem();

float updateLinearController(
	ControlMode mode,
	float position,
	float velocity,
	float reference,
	float reference_velocity,
	float reference_acceleration
);

float updateHeadingController(
	ControlMode mode,
	float heading,
	float angular_velocity,
	float reference_heading,
	float reference_angular_velocity,
	float reference_angular_acceleration
);


// ============================================================
// INITIALIZATION
// ============================================================

void initializeControlSystem();
void ensureControlInitialized();


#endif // MOTION_H