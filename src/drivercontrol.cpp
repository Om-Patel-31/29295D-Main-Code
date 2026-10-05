#include "drivercontrol.h"
#include <cmath>

// ============================================================
// GLOBAL STATE
// ============================================================

ScoringZone scoring_zone;
DriverState driver_state;


// ============================================================
// CENTER AND RADIUS CALCULATION
//
// Given 4 cardinal points (clockwise from North),
// calculate the center and radius of the circle.
//
// Index mapping:
// 0 = North (top):     (Cx,     Cy + R)
// 1 = East (right):    (Cx + R, Cy)
// 2 = South (bottom):  (Cx,     Cy - R)
// 3 = West (left):     (Cx - R, Cy)
//
// Center = average of all 4 points
// Radius = distance from center to any point
// ============================================================

void ScoringZone::calculate_center_and_radius()
{
	// Calculate center as average of all 4 points
	center.x = (base_points[0].x + base_points[1].x + 
	            base_points[2].x + base_points[3].x) * 0.25f;
	
	center.y = (base_points[0].y + base_points[1].y + 
	            base_points[2].y + base_points[3].y) * 0.25f;
	
	// Calculate radius as distance from center to first point
	Vec2 to_point = base_points[0] - center;
	base_radius = magnitude(to_point);
}


// ============================================================
// POINT-IN-QUADRILATERAL TEST
//
// Uses cross product method: point is inside if it's on the
// same side of all four edges.
// ============================================================

static bool is_point_in_quadrilateral(
	Vec2 point,
	const Vec2 quad[4]
)
{
	// For each edge of the quadrilateral, check if point is on inside
	for (int i = 0; i < 4; ++i)
	{
		Vec2 edge_start = quad[i];
		Vec2 edge_end = quad[(i + 1) % 4];
		
		// Vector from edge start to point
		Vec2 to_point = point - edge_start;
		
		// Edge vector
		Vec2 edge = edge_end - edge_start;
		
		// Cross product (2D): edge × to_point
		// If all cross products have same sign, point is inside
		float cross_product = cross(edge, to_point);
		
		// For clockwise quad, cross product should be negative (right side)
		if (cross_product > eps)
			return false;  // Point is outside this edge
	}
	
	return true;  // Point is inside all edges
}


// ============================================================
// SCORING ZONE CONFIGURATION
// ============================================================

void ScoringZone::configure(const Vec2 points[4], float offset)
{
	// Copy the 4 base points
	for (int i = 0; i < 4; ++i)
	{
		base_points[i] = points[i];
	}
	
	// Auto-calculate center and radius from the points
	calculate_center_and_radius();
	
	// Set the trigger offset
	trigger_offset = offset;
	
	// Calculate trigger points with offset
	update_trigger_points();
}


void ScoringZone::update_trigger_points()
{
	// Expand each base point radially outward from center by trigger_offset
	float trigger_radius = base_radius + trigger_offset;
	
	for (int i = 0; i < 4; ++i)
	{
		// Vector from center to base point
		Vec2 to_point = base_points[i] - center;
		
		// Normalize to get direction
		Vec2 direction = normalize(to_point);
		
		// Scale by trigger radius and add to center
		trigger_points[i] = center + direction * trigger_radius;
	}
}


bool ScoringZone::is_robot_in_trigger_zone(const Pose& pose)
{
	Vec2 robot_pos{pose.x, pose.y};
	return is_point_in_quadrilateral(robot_pos, trigger_points);
}


void ScoringZone::reset()
{
	is_active = false;
	sequence_executing = false;
	entry_time = 0.0f;
}


// ============================================================
// DRIVER CONTROL FUNCTIONS (from controls.cpp)
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


// ============================================================
// DRIVER CONTROL WITH SEMI-AUTONOMOUS INTEGRATION
// ============================================================

void driver_control_update(float forward, float turn)
{
	// If semi-autonomous sequence is running, skip manual control
	if (driver_state.is_semi_auto && scoring_zone.sequence_executing)
	{
		// Sequence has control; don't update manual commands
		return;
	}

	// Normal driver control
	float forward_cmd = driver_axis(forward);
	float turn_cmd = driver_axis(turn);

	float target_left = forward_cmd - turn_cmd * driver_tuning.turn_gain / 10.0f;
	float target_right = forward_cmd + turn_cmd * driver_tuning.turn_gain / 10.0f;

	driver_state.left = driver_acceleration(
		target_left,
		driver_state.left,
		driver_tuning.acceleration_gain
	);

	driver_state.right = driver_acceleration(
		target_right,
		driver_state.right,
		driver_tuning.acceleration_gain
	);

	driver_state.left = clampf(driver_state.left, -100.0f, 100.0f);
	driver_state.right = clampf(driver_state.right, -100.0f, 100.0f);

	// Apply to motors
	LFM.spin(vex::forward, driver_state.left, vex::pct);
	LBM.spin(vex::forward, driver_state.left, vex::pct);
	RFM.spin(vex::forward, driver_state.right, vex::pct);
	RBM.spin(vex::forward, driver_state.right, vex::pct);
}


void reset_driver_control()
{
	driver_state.left = 0.0f;
	driver_state.right = 0.0f;
	driver_state.is_semi_auto = false;

	stopDrive();
}


void set_semi_auto_active(bool active)
{
	driver_state.is_semi_auto = active;
	
	if (!active)
	{
		scoring_zone.sequence_executing = false;
	}
}


// ============================================================
// SCORING ZONE MANAGEMENT
// ============================================================

void configure_scoring_zone(
	const Vec2 points[4],
	float trigger_offset
)
{
	scoring_zone.configure(points, trigger_offset);
	scoring_zone.is_active = true;
}


void update_scoring_zone(const Pose& current_pose)
{
	// Only process if semi-auto is active
	if (!driver_state.is_semi_auto || !scoring_zone.is_active)
		return;

	// Check if robot is in trigger zone
	bool in_zone = scoring_zone.is_robot_in_trigger_zone(current_pose);

	// If we just entered the zone and not already executing
	if (in_zone && !scoring_zone.sequence_executing)
	{
		scoring_zone.sequence_executing = true;
		scoring_zone.entry_time = 0.0f;  // Could track time if needed
		
		// Start the semi-autonomous sequence
		execute_scoring_sequence();
	}
	// If we left the zone, stop execution
	else if (!in_zone && scoring_zone.sequence_executing)
	{
		scoring_zone.sequence_executing = false;
		stop_scoring_sequence();
	}
}


bool is_in_scoring_zone()
{
	return scoring_zone.sequence_executing;
}


void trigger_sequence()
{
	if (driver_state.is_semi_auto && scoring_zone.is_active)
	{
		scoring_zone.sequence_executing = true;
		execute_scoring_sequence();
	}
}


void stop_sequence()
{
	scoring_zone.sequence_executing = false;
	stop_scoring_sequence();
}


// ============================================================
// SEMI-AUTONOMOUS SEQUENCE EXECUTION
//
// This section is intentionally left blank for future development.
// The actual scoring sequence will be implemented here based on
// the specific game objectives and mechanism capabilities.
//
// The execute_scoring_sequence() function is called when the robot
// enters the trigger zone. It should:
//
// 1. Stop manual driver control (sequence has priority)
// 2. Execute the hardcoded sequence of movements and actions
// 3. Return control when sequence completes or stop_scoring_sequence() is called
//
// The stop_scoring_sequence() function should:
// 1. Cleanly stop any ongoing sequence operations
// 2. Return motors to neutral state
// 3. Restore manual driver control
//
// ============================================================

void execute_scoring_sequence()
{
	// TODO: Implement the actual scoring sequence
	//
	// This will be a hardcoded sequence specific to the game
	// and robot mechanisms. Placeholder for future development.
	//
	// Example structure:
	// 1. Move to scoring position
	// 2. Perform intake/outtake actions
	// 3. Deposit game objects
	// 4. Return to neutral position
	//
	// Sequence should use the motion control functions from motion.cpp
	// or direct motor commands as needed.
}


void stop_scoring_sequence()
{
	// TODO: Implement sequence cleanup
	//
	// This should:
	// 1. Stop any ongoing subsystem operations
	// 2. Bring robot to safe state
	// 3. Reset motors to neutral
	//
	// Placeholder for future development.
}