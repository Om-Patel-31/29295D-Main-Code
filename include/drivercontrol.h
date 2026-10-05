#ifndef DRIVERCONTROL_H
#define DRIVERCONTROL_H

#include "variables.h"

// ============================================================
// SEMI-AUTONOMOUS SCORING ZONE
// ============================================================

struct ScoringZone
{
	// Zone configuration - provided by user
	Vec2 base_points[4]{};             // Four cardinal points (clockwise from North)
	                                    // Index 0: North, 1: East, 2: South, 3: West
	
	// Auto-calculated from base points
	Vec2 center{0.0f, 0.0f};           // Calculated center of the zone
	float base_radius = 0.0f;          // Calculated radius from center to points
	
	// Tuning parameter
	float trigger_offset = 4.0f;       // Radial offset for trigger boundary // needs tuning. figure out how.
	
	// Trigger points (expanded with offset)
	Vec2 trigger_points[4]{};          // Trigger radius points (with offset)
	
	// Semi-autonomous state
	bool is_active = false;            // Zone is active for semi-auto
	bool sequence_executing = false;   // Currently running sequence
	float entry_time = 0.0f;           // Time robot entered zone
	
	// Methods
	void configure(const Vec2 points[4], float offset);
	void calculate_center_and_radius();
	void update_trigger_points();
	bool is_robot_in_trigger_zone(const Pose& pose);
	void reset();
};

extern ScoringZone scoring_zone;


// ============================================================
// DRIVER CONTROL STATE
// ============================================================

struct DriverState
{
	float left = 0.0f;
	float right = 0.0f;
	bool is_semi_auto = false;  // Currently under semi-auto control
};

extern DriverState driver_state;


// ============================================================
// DRIVER CONTROL FUNCTIONS
// ============================================================

// Manual driver control with semi-autonomous integration
void driver_control_update(float forward, float turn);

// Reset driver control state
void reset_driver_control();

// Enable/disable semi-autonomous mode
void set_semi_auto_active(bool active);


// ============================================================
// SEMI-AUTONOMOUS SCORING ZONE MANAGEMENT
// ============================================================

// Configure the scoring zone with 4 cardinal points
// Points should be in clockwise order: North, East, South, West
// Center and radius are auto-calculated from the points
void configure_scoring_zone(
	const Vec2 points[4],
	float trigger_offset
);

// Check if robot is in trigger zone and execute if needed
void update_scoring_zone(const Pose& current_pose);

// Get current zone status
bool is_in_scoring_zone();

// Manually trigger the semi-autonomous sequence
void trigger_sequence();

// Stop the semi-autonomous sequence
void stop_sequence();


// ============================================================
// SEMI-AUTONOMOUS SEQUENCE EXECUTION
//
// This is left blank and will be implemented later.
// Called when robot enters the trigger zone.
// ============================================================

void execute_scoring_sequence();

void stop_scoring_sequence();


// ============================================================
// UTILITY FUNCTIONS
// ============================================================

// Check if point is inside a quadrilateral (4 trigger points)
static bool is_point_in_quadrilateral(
	Vec2 point,
	const Vec2 quad[4]
);


#endif // DRIVERCONTROL_H