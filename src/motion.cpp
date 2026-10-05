#include "motion.h"
#include <cmath>

// ============================================================
// GLOBAL CONTROLLER INSTANCES
// ============================================================

ladrcState ladDist;
ladrcState ladHead;

pidState pidDist;
pidState pidHead;

BezierState bezier;


// ============================================================
// TRIGONOMETRIC MOTION PROFILE
// ============================================================

float ProfileState::A_p() const
{
	return (D * pi * pi) / (2.0f * T * T);
}


void ProfileState::reset(float distance, float duration)
{
	D = distance;
	T = std::fmax(duration, dt);

	s = 0.0f;
	v = 0.0f;
	a = A_p();
	j = 0.0f;

	rdot = 0.0f;
	rddot = 0.0f;

	time = 0.0f;
	finished = false;
}


void ProfileState::update(float elapsed)
{
	if (elapsed <= 0.0f)
		return;

	time += elapsed;

	float t = clampf(time / T, 0.0f, 1.0f);
	float angle = pi * t;

	s = 0.5f * D * (1.0f - std::cos(angle));
	v = D * pi / (2.0f * T) * std::sin(angle);
	a = D * pi * pi / (2.0f * T * T) * std::cos(angle);
	j = -D * pi * pi * pi / (2.0f * T * T * T) * std::sin(angle);

	rdot = v;
	rddot = a;

	if (time >= T)
	{
		time = T;
		s = D;
		v = 0.0f;
		a = 0.0f;
		j = 0.0f;
		rdot = 0.0f;
		rddot = 0.0f;
		finished = true;
	}
}


// ============================================================
// LADRC IMPLEMENTATION
// ============================================================

float ladrcState::beta1() const
{
	return 3.0f * w_o;
}


float ladrcState::beta2() const
{
	return 3.0f * w_o * w_o;
}


float ladrcState::beta3() const
{
	return w_o * w_o * w_o;
}


float ladrcState::k_p() const
{
	return w_c * w_c;
}


float ladrcState::k_d() const
{
	return 2.0f * w_c;
}


void ladrcState::reset(float y0)
{
	z1 = y0;
	z2 = 0.0f;
	z3 = 0.0f;

	e = 0.0f;
	u = 0.0f;
}


float ladrcState::update(float y, float r, float rd, float rdd)
{
	// ESO estimation error.
	e = z1 - y;

	// Third-order ESO.
	z1 += dt * (z2 - beta1() * e);
	z2 += dt * (z3 + b0 * u - beta2() * e);
	z3 += dt * (-beta3() * e);

	// Tracking error.
	float position_error = r - z1;
	float velocity_error = rd - z2;

	float u0 = k_p() * position_error + k_d() * velocity_error + rdd;

	// Disturbance rejection.
	float un = (u0 - z3) / b0;

	u = clampf(un, -v_cmd, v_cmd);

	return u;
}


// ============================================================
// PID IMPLEMENTATION
// ============================================================

void pidState::configure(const Plant& p, float w_c, float ki_scale)
{
	k_S = p.k_S;
	k_V = p.k_V;
	k_A = p.k_A;

	float b0 = p.b0();

	k_P = w_c * w_c / b0;
	k_D = 2.0f * w_c / b0;
	k_I = ki_scale * k_P * w_c;
}


void pidState::reset()
{
	integ = 0.0f;
	e = 0.0f;
}


float pidState::update(float y, float velocity, float r, float rd, float rdd)
{
	e = r - y;

	// Feedforward.
	float ff = k_S * sgnf(rd) + k_V * rd + k_A * rdd;

	// Feedback.
	float un = ff + k_P * e + k_I * integ + k_D * (rd - velocity);

	// Conditional integration.
	if (std::fabs(un) < v_cmd && k_I > 0.0f)
	{
		integ = clampf(integ + e * dt, -3.0f / k_I, 3.0f / k_I);
	}

	return clampf(un, -v_cmd, v_cmd);
}


// ============================================================
// CUBIC BEZIER CURVE IMPLEMENTATION
//
// Cubic Bezier:
//   B(t) = (1-t)^3 * P0 + 3(1-t)^2*t * P1 +
//           3(1-t)*t^2 * P2 + t^3 * P3
//
// where P0 = start, P3 = end, P1 and P2 are control points
// ============================================================

static float B0(float t)
{
	float mt = 1.0f - t;
	return mt * mt * mt;
}


static float B1(float t)
{
	float mt = 1.0f - t;
	return 3.0f * mt * mt * t;
}


static float B2(float t)
{
	float mt = 1.0f - t;
	return 3.0f * mt * t * t;
}


static float B3(float t)
{
	return t * t * t;
}


static float dB0(float t)
{
	float mt = 1.0f - t;
	return -3.0f * mt * mt;
}


static float dB1(float t)
{
	float mt = 1.0f - t;
	return 3.0f * (mt * mt - 2.0f * mt * t);
}


static float dB2(float t)
{
	float mt = 1.0f - t;
	return 3.0f * (2.0f * mt * t - t * t);
}


static float dB3(float t)
{
	return 3.0f * t * t;
}


static float ddB0(float t)
{
	return 6.0f * (t - 1.0f);
}


static float ddB1(float t)
{
	return 6.0f * (1.0f - 2.0f * t);
}


static float ddB2(float t)
{
	return 6.0f * (2.0f * t - 1.0f);
}


static float ddB3(float t)
{
	return 6.0f * t;
}


Vec2 bezierPosition(const BezierState& curve, float t)
{
	t = clampf(t, 0.0f, 1.0f);

	return curve.p0 * B0(t) + curve.p1 * B1(t) +
		   curve.p2 * B2(t) + curve.p3 * B3(t);
}


Vec2 bezierDerivative(const BezierState& curve, float t)
{
	t = clampf(t, 0.0f, 1.0f);

	return curve.p0 * dB0(t) + curve.p1 * dB1(t) +
		   curve.p2 * dB2(t) + curve.p3 * dB3(t);
}


Vec2 bezierSecondDerivative(const BezierState& curve, float t)
{
	t = clampf(t, 0.0f, 1.0f);

	return curve.p0 * ddB0(t) + curve.p1 * ddB1(t) +
		   curve.p2 * ddB2(t) + curve.p3 * ddB3(t);
}


float bezierCurvature(const BezierState& curve, float t)
{
	Vec2 d1 = bezierDerivative(curve, t);
	Vec2 d2 = bezierSecondDerivative(curve, t);

	float denominator = std::pow(dot(d1, d1), 1.5f);

	if (denominator < eps)
		return 0.0f;

	return cross(d1, d2) / denominator;
}


void buildBezierArcTable()
{
	bezier.s_table[0] = 0.0f;

	float previous_length = 0.0f;

	for (int i = 1; i <= BezierState::N; ++i)
	{
		float t0 = static_cast<float>(i - 1) / static_cast<float>(BezierState::N);
		float t1 = static_cast<float>(i) / static_cast<float>(BezierState::N);

		Vec2 p0 = bezierPosition(bezier, t0);
		Vec2 p1 = bezierPosition(bezier, t1);

		float ds = magnitude(p1 - p0);

		previous_length += ds;

		bezier.s_table[i] = previous_length;
	}

	bezier.L = bezier.s_table[BezierState::N];
}


void configureBezier(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3)
{
	bezier.p0 = p0;
	bezier.p1 = p1;
	bezier.p2 = p2;
	bezier.p3 = p3;

	buildBezierArcTable();

	bezier.kappa_max = 0.0f;

	for (int i = 0; i <= BezierState::N; ++i)
	{
		float t = static_cast<float>(i) / static_cast<float>(BezierState::N);

		float kappa = std::fabs(bezierCurvature(bezier, t));

		if (kappa > bezier.kappa_max)
			bezier.kappa_max = kappa;
	}
}


float arcLengthToParameter(const BezierState& curve, float s)
{
	if (curve.L <= eps)
		return 0.0f;

	s = clampf(s, 0.0f, curve.L);

	for (int i = 1; i <= BezierState::N; ++i)
	{
		if (s <= curve.s_table[i])
		{
			float s0 = curve.s_table[i - 1];
			float s1 = curve.s_table[i];

			float local = (s - s0) / std::fmax(s1 - s0, eps);

			float t0 = static_cast<float>(i - 1) / static_cast<float>(BezierState::N);
			float t1 = static_cast<float>(i) / static_cast<float>(BezierState::N);

			return lerpf(t0, t1, local);
		}
	}

	return 1.0f;
}


// ============================================================
// PATH ERRORS
// ============================================================

PathErrors calculatePathErrors(
	const BezierState& curve,
	const Pose& pose,
	float t
)
{
	PathErrors errors;

	t = clampf(t, 0.0f, 1.0f);

	Vec2 target = bezierPosition(curve, t);
	Vec2 derivative = bezierDerivative(curve, t);
	Vec2 tangent = normalize(derivative);

	Vec2 position{pose.x, pose.y};
	Vec2 error = target - position;

	errors.e_along = dot(error, tangent);
	errors.e_cross = cross(tangent, error);

	float target_heading = std::atan2(tangent.y, tangent.x);

	errors.e_theta = wrapAngle(target_heading - pose.theta);

	errors.k_theta = 2.0f;		// needs tuning. figure out how.
	errors.k_c = 2.0f;			// needs tuning. figure out how.

	return errors;
}


float projectPoseOntoCurve(const BezierState& curve, const Pose& pose)
{
	Vec2 robot{pose.x, pose.y};

	float best_t = 0.0f;
	float best_distance = 1.0e30f;

	for (int i = 0; i <= BezierState::N; ++i)
	{
		float t = static_cast<float>(i) / static_cast<float>(BezierState::N);

		Vec2 point = bezierPosition(curve, t);
		Vec2 error = point - robot;

		float distance = dot(error, error);

		if (distance < best_distance)
		{
			best_distance = distance;
			best_t = t;
		}
	}

	return best_t;
}


// ============================================================
// DIFFERENTIAL DRIVE MIXER
// ============================================================

DriveOutput mixDrive(float linear, float angular)
{
	DriveOutput output;

	output.left = linear - angular;
	output.right = linear + angular;

	float maximum = std::fmax(1.0f,
		std::fmax(std::fabs(output.left), std::fabs(output.right)));

	if (maximum > v_cmd)
	{
		output.left /= maximum / v_cmd;
		output.right /= maximum / v_cmd;
	}

	output.left = clampf(output.left, -v_cmd, v_cmd);
	output.right = clampf(output.right, -v_cmd, v_cmd);

	return output;
}


// ============================================================
// CONTROLLER MANAGEMENT
// ============================================================

void applyGains(float w_c, float w_o, float ki_scale)
{
	ladDist.w_c = w_c;
	ladHead.w_c = w_c;

	ladDist.w_o = w_o;
	ladHead.w_o = w_o;

	ladDist.b0 = linearPlant.b0();
	ladHead.b0 = turnPlant.b0();

	pidDist.configure(linearPlant, w_c, ki_scale);
	pidHead.configure(turnPlant, w_c, ki_scale);
}


void resetControllers()
{
	ladDist.reset(0.0f);
	ladHead.reset(0.0f);

	pidDist.reset();
	pidHead.reset();
}


void resetControlSystem()
{
	resetOdometry();
	resetControllers();
	setDriveCommand(0.0f, 0.0f);
}


float updateLinearController(
	ControlMode mode,
	float position,
	float velocity,
	float reference,
	float reference_velocity,
	float reference_acceleration
)
{
	if (mode == ControlMode::LADRC)
	{
		return ladDist.update(position, reference, reference_velocity, reference_acceleration);
	}

	return pidDist.update(position, velocity, reference, reference_velocity, reference_acceleration);
}


float updateHeadingController(
	ControlMode mode,
	float heading,
	float angular_velocity,
	float reference_heading,
	float reference_angular_velocity,
	float reference_angular_acceleration
)
{
	reference_heading = wrapAngle(reference_heading);
	heading = wrapAngle(heading);

	if (mode == ControlMode::LADRC)
	{
		return ladHead.update(heading, reference_heading, reference_angular_velocity, reference_angular_acceleration);
	}

	return pidHead.update(heading, angular_velocity, reference_heading, reference_angular_velocity, reference_angular_acceleration);
}


// ============================================================
// INITIALIZATION
// ============================================================

void initializeControlSystem()
{
	applyGains(
		12.0f,	// controller bandwidth // needs tuning. figure out how.
		48.0f,	// observer bandwidth	// needs tuning. figure out how.
		0.05f	// integral scale		// needs tuning. figure out how.
	);

	resetControlSystem();
}


void ensureControlInitialized()
{
	if (!control_initialized)
	{
		initializeControlSystem();
		control_initialized = true;
	}
}