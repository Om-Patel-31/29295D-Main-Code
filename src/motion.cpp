#include "motion.h"  // Include autonomous motion engine and DriveController declarations
#include "drivetrain.h" // Include shared chassis output layer
#include "robot.h"   // Include robot hardware extern declarations and measurement helpers
#include "tuning.h"  // Include centralized tuning configuration parameters
#include "ladrc2.h"  // Include LADRC controller and ESO observer
#include "odometry.h"// Include Odometry and measurement structures
#include "quintic.h" // Include QuinticProfile and Quintic limits
#include "vex.h"     // Include VEX V5 Robotics library API
#include "telemetry.h" // Include CSV telemetry logger
#include <algorithm> // Include std::min, std::max, std::fabs
#include <cmath>     // Include mathematical functions for trigonometry

using namespace vex; // Expose VEX SDK classes and enums

namespace {          // Anonymous namespace for internal path tracking helpers
// Computes safe available battery voltage, clamping to configured max limit
double AvailableDriveVoltage(double supply_voltage)
{
    if (!std::isfinite(supply_voltage) || supply_voltage <= 0.0) { // Check if battery voltage reading is valid
        return tuning.max_voltage;                                // Default to safe maximum system voltage
    }
    return std::min(tuning.max_voltage, supply_voltage);          // Clamp supply voltage to upper system limit
}

// Measures traveled distance along the Bezier path with unbounded tangential extrapolation
double MeasurePathDistance(const BezierPath& path, const Point2& point)
{
    const double s = path.ProjectDistance(point);                 // Find nearest equidistant sample distance
    const Point2 nearest = path.PointAt(s);                       // Evaluate path coordinate at sampled distance
    const double heading = path.HeadingAt(s);                     // Evaluate tangent heading at sampled distance
    const double along = (point.x - nearest.x) * std::cos(heading) + // Project residual offset onto tangent unit vector
                         (point.y - nearest.y) * std::sin(heading);
    const double spacing = path.length() / BezierPath::samples;   // Sample discretization spacing
    const double lo = s <= 0.0 ? -1e9 : -spacing;                 // Allow negative extrapolation past path start
    const double hi = s >= path.length() ? 1e9 : spacing;         // Allow positive extrapolation past path end
    return s + std::max(lo, std::min(hi, along));                 // Return refined continuous distance
}
} // namespace

// Wraps an angle to the closest continuous branch near a reference heading to eliminate +/-180 wrap discontinuities
double WrapToNear(double angle, double reference)
{
    const double pi = 3.14159265358979323846;                     // Ratio of circumference to diameter
    return reference + std::remainder(angle - reference, 2.0 * pi); // Select nearest equivalent modular angle
}

// Default constructor initializing longitudinal and heading LADRC from tuning parameters
DriveController::DriveController()
    : path_(),                                                    // Initialize blank path geometry
      profile_(),                                                 // Initialize blank quintic profile
      longitudinal_(tuning.drive_wc(), tuning.drive_b0_mm(),      // Configure linear LADRC bandwidth and plant gain
                    tuning.drive_wo(), tuning.drive_max_disturb_mm()), // Configure linear ESO bandwidth and disturbance clamp
      heading_(tuning.turn_wc(), tuning.turn_b0_rad(),            // Configure angular LADRC bandwidth and plant gain
               tuning.turn_wo(), tuning.turn_max_disturb_rad()),  // Configure angular ESO bandwidth and disturbance clamp
      elapsed_(0.0),                                              // Clear elapsed trajectory timer
      hold_time_(0.0),                                            // Clear settle hold timer
      settle_speed_(-1.0),                                        // Default linear settle speed threshold disabled
      settle_turn_rate_(-1.0),                                    // Default angular settle rate threshold disabled
      active_(false),                                             // Mark controller as initially inactive
      faulted_(false)                                             // Mark controller as not faulted
{                                                                 // Constructor body
}                                                                 // End constructor

// Custom constructor allowing explicit override of controller bandwidths and gains
DriveController::DriveController(double lwc, double lb0, double hwc, double hb0)
    : path_(),                                                    // Initialize blank path geometry
      profile_(),                                                 // Initialize blank quintic profile
      longitudinal_(lwc, lb0, lwc * tuning.drive_wo_ratio,        // Configure linear LADRC with explicit parameters
                    tuning.drive_max_disturb_mm()),               // Set linear disturbance clamp
      heading_(hwc, hb0, hwc * tuning.turn_wo_ratio,              // Configure angular LADRC with explicit parameters
               tuning.turn_max_disturb_rad()),                    // Set angular disturbance clamp
      elapsed_(0.0),                                              // Clear elapsed trajectory timer
      hold_time_(0.0),                                            // Clear settle hold timer
      settle_speed_(-1.0),                                        // Default linear settle speed threshold disabled
      settle_turn_rate_(-1.0),                                    // Default angular settle rate threshold disabled
      active_(false),                                             // Mark controller as initially inactive
      faulted_(false)                                             // Mark controller as not faulted
{                                                                 // Constructor body
}                                                                 // End constructor

// Configures optional velocity thresholds required to declare motion settled
void DriveController::SetSettleSpeeds(double speed, double turn_rate)
{
    settle_speed_ = speed;                                        // Store linear velocity settle threshold (mm/s)
    settle_turn_rate_ = turn_rate;                                // Store angular rate settle threshold (rad/s)
}

// Begins execution of a new Bezier path trajectory using a quintic minimum-jerk profile
bool DriveController::StartPath(const BezierPath& path, const Pose2& measured,
                                double vmax, double amax)
{
    active_ = false;                                              // Reset active status prior to validation
    faulted_ = false;                                             // Clear any previous fault flag
    if (!path.valid() ||                                          // Verify path geometry is mathematically valid
        !ValidateQuinticLimits(amax, longitudinal_.input_gain(),  // Validate profile acceleration against motor capability
                               longitudinal_.bandwidth(), tuning.max_voltage))
        return false;                                             // Reject invalid path or limits

    path_ = path;                                                 // Store valid target path
    const double kappa = path_.max_curvature();                   // Extract maximum curvature along the path
    const double curvature_limit = kappa > 0.0                    // Compute centripetal lateral acceleration speed limit: v = sqrt(a_lat / kappa)
        ? std::sqrt(tuning.lateral_accel_max_mm() / kappa)
        : vmax;
    const double effective_vmax = std::min(vmax, curvature_limit);// Clamp maximum velocity to preserve lateral traction
    active_ = profile_.Configure(path_.length(), effective_vmax, amax); // Generate quintic minimum-jerk profile
    elapsed_ = 0.0;                                               // Reset elapsed motion timer
    hold_time_ = 0.0;                                             // Reset settle tolerance timer
    const double projected = path_.ProjectDistance({measured.x, measured.y}); // Find initial distance projection
    const double s0 = std::max(0.0, std::min(path_.length(), projected)); // Clamp initial distance to path bounds
    const double heading_reference = path_.HeadingAt(s0);          // Look up path tangent heading at starting point
    longitudinal_.Reset(s0, 0.0);                                 // Reset linear ESO to current starting displacement
    heading_.Reset(WrapToNear(measured.heading, heading_reference), 0.0); // Reset angular ESO to starting heading
    longitudinal_.SetAppliedInput(0.0);                           // Clear applied linear voltage
    heading_.SetAppliedInput(0.0);                                // Clear applied angular voltage
    return active_;                                               // Return true if trajectory started successfully
}

// Resets controller state and resets observer to specified robot pose
void DriveController::Reset(const Pose2& p)
{
    longitudinal_.Reset(0.0, 0.0);                                // Reset linear observer state to zero
    heading_.Reset(p.heading, 0.0);                               // Reset angular observer state to current heading
    longitudinal_.SetAppliedInput(0.0);                           // Clear previous applied linear voltage
    heading_.SetAppliedInput(0.0);                                // Clear previous applied angular voltage
    elapsed_ = 0.0;                                               // Clear elapsed motion timer
    hold_time_ = 0.0;                                             // Clear settle hold timer
    active_ = false;                                              // Mark trajectory inactive
    faulted_ = false;                                             // Clear fault state
}

// Executes one discrete control cycle, returning left and right drive wheel motor voltages
DriveVoltages DriveController::Update(double dt, const Pose2& measured, double supply_voltage)
{
    if (!active_) return {0.0, 0.0};                              // If trajectory is inactive, output zero volts
    const bool profile_done = elapsed_ >= profile_.duration();    // Check if theoretical trajectory profile is completed
    const double final_heading = path_.HeadingAt(path_.length()); // Target heading at path end point
    QuinticState ref = profile_done                               // Sample reference trajectory state or hold final state
        ? QuinticState{path_.length(), 0.0, 0.0, true}
        : profile_.Sample(elapsed_);
    double measured_s = MeasurePathDistance(path_, {measured.x, measured.y}); // Measure traveled distance along path
    double theta = path_.HeadingAt(ref.position);                 // Reference heading angle along path tangent
    double omega = ref.velocity * path_.CurvatureAt(ref.position);// Reference angular velocity from path curvature

    if (!profile_done) {                                          // Apply cross-track steering correction while cruising
        const double s_near = std::max(0.0, std::min(path_.length(), measured_s)); // Find nearest path distance
        const Point2 closest = path_.PointAt(s_near);             // Find nearest point coordinates on curve
        const double h_near = path_.HeadingAt(s_near);            // Tangent direction at nearest point
        const double e = -(measured.x - closest.x) * std::sin(h_near) + // Compute perpendicular cross-track error
                          (measured.y - closest.y) * std::cos(h_near);
        const double fade = std::min(1.0, std::fabs(ref.velocity) / 50.0); // Fade steering correction to 0 at low velocities
        const double raw_correction = tuning.cross_track_gain_rad_per_mm() * e; // Compute proportional heading correction
        const double max_corr = tuning.cross_track_max_rad();     // Maximum allowed cross-track heading correction clamp
        const double correction = std::max(-max_corr, std::min(max_corr, raw_correction)); // Clamp steering correction
        theta -= fade * correction;                               // Adjust heading reference toward path centerline
    }

    double u_long = longitudinal_.Update(dt, measured_s, ref.position, ref.velocity, ref.acceleration); // Update linear LADRC
    const double measured_heading = WrapToNear(measured.heading, theta); // Wrap measured heading to avoid 360 wrap
    double u_turn = heading_.Update(dt, measured_heading, theta, omega, 0.0); // Update heading LADRC
    if (!longitudinal_.valid() || !heading_.valid()) {            // Detect numerical failure in LADRC loops
        faulted_ = true;                                          // Mark controller as faulted
        active_ = false;                                          // Halt path execution
        return {0.0, 0.0};                                        // Return safe zero voltage command
    }

    if (profile_done) {                                           // Trajectory profile has reached destination
        const double position_error = std::fabs(path_.length() - measured_s); // Measure linear distance error
        const double heading_error = std::fabs(                   // Measure angular heading error
            WrapToNear(measured.heading, final_heading) - final_heading);
        const bool slow =                                         // Verify robot has decelerated below settle speed thresholds
            (settle_speed_ < 0.0 ||
             std::fabs(longitudinal_.velocity_estimate()) <= settle_speed_) &&
            (settle_turn_rate_ < 0.0 ||
             std::fabs(heading_.velocity_estimate()) <= settle_turn_rate_);
        if (position_error <= tuning.drive_tolerance_mm() &&      // Verify robot is within linear tolerance
            heading_error <= tuning.turn_tolerance_rad() && slow) { // Verify robot is within angular tolerance and stopped
            hold_time_ += dt;                                     // Accumulate settled hold duration
        } else {
            hold_time_ = 0.0;                                     // Reset settle timer if robot was knocked off target
        }
        if (hold_time_ >= tuning.settle_hold_time_s) {             // Confirm robot remained stable for required hold time
            active_ = false;                                      // Motion complete: terminate active tracking
        }

        // NOTE: We no longer call stop_drive() here - the LADRC controller
        // naturally decelerates to zero at the end of each trajectory.
        // The caller (drive/turn/bdrive) is responsible for braking if needed.
        // Removing stop_drive() from here enables smooth chaining without stepping.
    }

    const double voltage_limit = AvailableDriveVoltage(supply_voltage); // Determine maximum available battery voltage
    const double left_request = u_long + u_turn;                  // Mix linear and turn voltage requests for left side
    const double right_request = u_long - u_turn;                 // Mix linear and turn voltage requests for right side
    const double peak = std::max(std::fabs(left_request), std::fabs(right_request)); // Find maximum requested motor command
    const bool saturated = peak > voltage_limit;                  // Detect if requested voltage exceeds available supply
    const double scale = saturated ? voltage_limit / peak : 1.0;  // Scale both sides proportionally to preserve turn radius
    const double left = left_request * scale;                     // Scaled left motor voltage
    const double right = right_request * scale;                   // Scaled right motor voltage
    longitudinal_.SetAppliedInput(0.5 * (left + right));          // Feed back actual common-mode voltage to linear ESO
    heading_.SetAppliedInput(0.5 * (left - right));               // Feed back actual differential voltage to heading ESO
    if (!profile_done && !saturated) elapsed_ += dt;              // Advance profile time only when actuators are not saturated
    return {left, right};                                         // Return final drive motor voltages
}

// ============================================================================
// Hardware Autonomous Driving Implementation Helpers
// ============================================================================

namespace {
const double kPi = 3.14159265358979323846;                         // Mathematical constant pi
const double kDegToRad = kPi / 180.0;                              // Conversion factor from degrees to radians
const double kMmPerIn = 25.4;                                      // Conversion factor from inches to millimeters

// Actively stops all 4 drivetrain motors using brake mode to prevent coasting
void stop_drive()
{
    chassis.Stop(coast);                                           // Brake both sides through the shared output layer
}

// Commands voltages to left and right drivetrain sides
void set_drive(double left_volts, double right_volts)
{
    chassis.SetVoltage(left_volts, right_volts);                   // Both sides through the shared output layer
}

// Halts drivetrain and displays diagnostic failure notice on V5 Brain screen
void show_failure(const char* message)
{
    stop_drive();                                                  // Halt motors immediately
    Brain.Screen.clearScreen();                                    // Clear Brain screen
    Brain.Screen.setPenColor(color(255, 100, 100));                // Set warning text color
    Brain.Screen.printAt(20, 120, message);                        // Print failure message at coordinate (20, 120)
}

// Normalizes speed percentage input into a scaling fraction bounded in [0.01, 1.0]
double speed_scale(double speed_pct)
{
    if (!std::isfinite(speed_pct)) return 1.0;                     // Default to 100% on invalid input
    return std::max(0.01, std::min(1.0, speed_pct / 100.0));       // Clamp speed factor between 1% and 100%
}

// Reads available battery voltage, capped at the configured maximum system voltage
double available_voltage()
{
    const double battery = Brain.Battery.voltage(volt);            // Query V5 Brain battery voltage
    if (!std::isfinite(battery) || battery <= 0.0) {               // Check if battery reading is invalid
        return tuning.max_voltage;                                 // Fall back to configured voltage limit
    }
    return std::min(tuning.max_voltage, battery);                  // Return the smaller of supply or voltage limit
}

// Reads continuous, unbounded inertial rotation converted to clockwise-positive radians
double imu_heading_rad()
{
    return imu.rotation(degrees) * kDegToRad;                      // Multiply degrees by deg-to-rad factor
}

// Gathers current sensor readings into an OdometryMeasurement packet in SI units (mm, rad)
OdometryMeasurement read_measurement()
{
    const double mm_per_deg = tuning.tracking_wheel_circ_mm() / 360.0; // Millimeters per tracking wheel degree
    return {
        average_left_mm(),                                         // Average left chassis travel in mm
        average_right_mm(),                                        // Average right chassis travel in mm
        tracking_parallel.position(degrees) * mm_per_deg,          // Parallel tracking wheel travel in mm
        tuning.use_horizontal_tracking_wheel                       // Lateral tracking wheel travel in mm if enabled
            ? tracking_perp.position(degrees) * mm_per_deg : 0.0,
        imu_heading_rad(),                                         // Inertial heading in radians
        imu.gyroRate(zaxis, dps) * kDegToRad                       // Inertial angular velocity in rad/s
    };
}

// Determines execution timeout, using user input if specified or planned duration + safety margin
double timeout_for(double timeout_s, double planned_s)
{
    return timeout_s > 0.0 ? timeout_s : planned_s + tuning.motion_timeout_margin_s; // Return active timeout limit
}

// Pauses execution for one control loop period and calculates measured loop delta time in seconds
double wait_tick(timer& clock, double& last_s)
{
    const int loop_ms = static_cast<int>(1000.0 / tuning.control_hz + 0.5); // Loop duration in milliseconds
    wait(loop_ms, msec);                                           // Pause execution for control period
    const double now = clock.time(sec);                            // Query current timer timestamp in seconds
    const double dt = now - last_s;                                // Calculate delta time since previous tick
    last_s = now;                                                  // Update previous timestamp
    return std::max(0.001, std::min(0.05, dt));                    // Clamp delta time to avoid timer spikes
}

// Transforms robot pose into path frame: inverts coordinates for reverse trajectories
Pose2 to_path_frame(const Pose2& pose, bool reverse)
{
    if (!reverse) return pose;                                     // Forward moves use standard pose
    return {-pose.x, -pose.y, pose.heading};                       // Reverse moves invert positional coordinates
}

// Core path-following engine: executes a Bezier curve defined by endpoint relative to current pose
bool follow_path(double dx, double dy, double end_heading_rad, double speed_pct, double timeout_s)
{
    const bool reverse = dx < 0.0;                                 // Reverse trajectory if longitudinal offset is negative
    BezierPath path;                                               // Instantiate Bezier path geometry
    if (!path.Configure(reverse ? -dx : dx, reverse ? -dy : dy,    // Build cubic Bezier curve with curvature limits
                        end_heading_rad, tuning.min_turn_radius_mm())) {
        show_failure("Auton path invalid");                        // Display error if curve generation failed
        return false;                                              // Return false to indicate motion failure
    }

    Odometry odom(tuning.use_horizontal_tracking_wheel             // Instantiate fresh local odometry instance
                      ? tuning.horizontal_wheel_offset_mm() : 0.0);
    const OdometryMeasurement initial = read_measurement();        // Sample initial robot state
    odom.Reset(initial, initial.inertial_heading);                 // Tare local odometry frame to current pose

    DriveController controller;                                    // Instantiate dual-LADRC path controller
    controller.SetSettleSpeeds(tuning.settle_speed_mm_s(),         // Set linear settle speed threshold
                               tuning.settle_turn_rate_rad_s());   // Set angular settle rate threshold
    if (!controller.StartPath(path, to_path_frame(odom.pose(), reverse), // Initialize trajectory profile
                              tuning.v_max_mm_s() * speed_scale(speed_pct), // Scale max cruise speed
                              tuning.a_max_mm_s2())) {             // Set acceleration limits
        show_failure("Auton drive limits invalid");                // Display error if limits violated motor limits
        return false;                                              // Return false on limit error
    }

    const unsigned int log_segment = TelemetryBeginSegment();      // Tag data for this autonomous motion segment
    const double limit_s = timeout_for(timeout_s, controller.profile().duration()); // Compute abort timeout
    timer clock;                                                   // Create loop timer
    double last_s = clock.time(sec);                               // Initialize last timestamp
    while (controller.active() && clock.time(sec) < limit_s) {     // Iterate while trajectory is active and within timeout
        const double dt = wait_tick(clock, last_s);                // Wait for next control tick
        const Pose2 pose = to_path_frame(odom.Update(dt, read_measurement()), reverse); // Update local pose
        const double sample_profile_time = controller.elapsed(); // Reference timestamp for this measured sample
        const DriveVoltages out = controller.Update(dt, pose, available_voltage()); // Run controller update
        if (controller.faulted()) {                                // Check for controller numerical fault
            show_failure("Auton drive fault");                     // Display fault message
            return false;                                          // Abort on fault
        }
        if (reverse) set_drive(-out.right, -out.left);             // Swap and invert sides when driving in reverse
        else set_drive(out.left, out.right);                       // Apply normal forward motor voltages
        if (TelemetryActive()) {
            const QuinticState ref = controller.profile().Sample(sample_profile_time);
            const Point2 ref_point = path.PointAt(ref.position);
            const double ref_heading = path.HeadingAt(ref.position);
            const double measured_s = MeasurePathDistance(path, {pose.x, pose.y});
            const Point2 closest = path.PointAt(path.ProjectDistance({pose.x, pose.y}));
            const double cross_track = -(pose.x - closest.x) * std::sin(ref_heading) +
                                       (pose.y - closest.y) * std::cos(ref_heading);
            const double left_applied = reverse ? -out.right : out.left;
            const double right_applied = reverse ? -out.left : out.right;
            TelemetrySample sample = {
                log_segment, "drive", true, TelemetryTime(), "in",
                ref.position / kMmPerIn, measured_s / kMmPerIn,
                ref.velocity / kMmPerIn,
                controller.longitudinal_velocity_estimate() / kMmPerIn,
                ref.acceleration / kMmPerIn,
                controller.longitudinal_disturbance_estimate() / kMmPerIn,
                "in/s", "in/s^2", controller.longitudinal_output(),
                Brain.Battery.voltage(volt), left_applied, right_applied,
                ref_heading / kDegToRad, pose.heading / kDegToRad,
                ref_point.x / kMmPerIn, ref_point.y / kMmPerIn,
                pose.x / kMmPerIn, pose.y / kMmPerIn,
                cross_track / kMmPerIn
            };
            TelemetryLog(sample);
        }
    }

    if (controller.active()) {                                     // If controller remained active, timeout occurred
        show_failure("Auton drive timeout");                       // Display timeout notice
        return false;                                              // Motion did not settle in time
    }
    return true;                                                   // Motion settled successfully within tolerances
    // NOTE: Motors naturally decay to zero via LADRC controller,
    // enabling smooth chaining without stepping/stopping between motions.
}

// Rotates the robot in place by a relative angle in radians using angular LADRC and quintic profile
bool turn_by(double angle_rad, double speed_pct, double timeout_s)
{
    if (!ValidateQuinticLimits(tuning.turn_a_max_rad_s2(),         // Validate turn acceleration limits
                               tuning.turn_b0_rad(), tuning.turn_wc(), tuning.max_voltage)) {
        show_failure("Auton turn limits invalid");                 // Display limit validation error
        return false;                                              // Abort turn on invalid limits
    }
    if (!std::isfinite(angle_rad)) {                               // Check if turn angle is valid
        show_failure("Auton turn angle invalid");                  // Display invalid angle error
        return false;                                              // Abort on NaN/Inf
    }
    if (std::fabs(angle_rad) < 1e-6) return true;                  // Trivial zero-angle turn completes immediately

    QuinticProfile profile;                                        // Instantiate angular quintic profile
    if (!profile.Configure(angle_rad, tuning.turn_v_max_rad_s() * speed_scale(speed_pct), // Plan quintic velocity profile
                           tuning.turn_a_max_rad_s2())) {          // Set angular acceleration limit
        show_failure("Auton turn profile invalid");                // Display profile calculation failure
        return false;                                              // Abort on profile failure
    }

    const double start = imu_heading_rad();                        // Record starting heading in radians
    const double target = start + angle_rad;                       // Compute target heading in radians
    Ladrc2 heading(tuning.turn_wc(), tuning.turn_b0_rad(),         // Instantiate heading LADRC controller
                   tuning.turn_wo(), tuning.turn_max_disturb_rad()); // Pass observer bandwidth and disturbance clamp
    heading.Reset(start, 0.0);                                     // Reset observer to initial heading
    heading.SetAppliedInput(0.0);                                  // Clear applied input voltage

    double elapsed = 0.0;                                          // Initialize elapsed profile duration
    double hold = 0.0;                                             // Initialize settled hold duration
    const unsigned int log_segment = TelemetryBeginSegment();      // Tag this turn in the CSV
    const double limit_s = timeout_for(timeout_s, profile.duration()); // Calculate abort timeout
    timer clock;                                                   // Create loop timer
    double last_s = clock.time(sec);                               // Initialize last timestamp
    while (clock.time(sec) < limit_s) {                            // Loop until settled or timeout expires
        const double dt = wait_tick(clock, last_s);                // Wait for next control tick
        const double measured = imu_heading_rad();                 // Read current heading from inertial sensor
        const bool profile_done = elapsed >= profile.duration();   // Check if profile duration has finished
        const QuinticState ref = profile_done                      // Sample trajectory reference state
            ? QuinticState{angle_rad, 0.0, 0.0, true}
            : profile.Sample(elapsed);

        const double u = heading.Update(dt, measured, start + ref.position, // Run LADRC heading update
                                        ref.velocity, ref.acceleration);
        if (!heading.valid()) {                                    // Detect numerical instability
            show_failure("Auton turn fault");                      // Display fault message
            return false;                                          // Abort on fault
        }

        const double limit = available_voltage();                  // Query available battery voltage limit
        const bool saturated = std::fabs(u) > limit;               // Check if commanded voltage saturates battery
        const double applied = std::max(-limit, std::min(limit, u)); // Clamp voltage command to supply limits
        set_drive(applied, -applied);                              // Apply differential turning voltage (CW = left+, right-)
        heading.SetAppliedInput(applied);                          // Feed back actual voltage to heading ESO
        if (TelemetryActive()) {
            const double measured_deg = (measured - start) / kDegToRad;
            const double measured_heading_deg = measured / kDegToRad;
            const double ref_deg = (start + ref.position) / kDegToRad;
            const double battery = Brain.Battery.voltage(volt);
            TelemetrySample sample = {
                log_segment, "turn", false, TelemetryTime(), "deg",
                ref.position / kDegToRad, measured_deg, ref.velocity / kDegToRad,
                heading.velocity_estimate() / kDegToRad,
                ref.acceleration / kDegToRad, heading.disturbance() / kDegToRad,
                "deg/s", "deg/s^2", applied, battery, applied, -applied,
                ref_deg, measured_heading_deg, 0.0, 0.0, 0.0, 0.0, 0.0
            };
            TelemetryLog(sample);
        }

        if (!profile_done && !saturated) elapsed += dt;            // Advance profile time when not saturated

        if (profile_done) {                                        // Evaluate settling criteria once profile ends
            const bool on_target =                                 // Check if heading error is within tolerance
                std::fabs(target - measured) <= tuning.turn_tolerance_rad();
            const bool stopped =                                   // Check if rotational rate is below settle threshold
                std::fabs(heading.velocity_estimate()) <= tuning.settle_turn_rate_rad_s();
            hold = on_target && stopped ? hold + dt : 0.0;         // Accumulate settled hold time or reset
            if (hold >= tuning.settle_hold_time_s) {               // Check if robot maintained tolerance long enough
                return true;                                       // Turn settled successfully (LADRC naturally decelerates)
            }
        }
    }

    show_failure("Auton turn timeout");                            // Display timeout notice
    return false;                                                  // Turn did not settle in time
    // NOTE: Motors naturally decay to zero via LADRC controller,
    // enabling smooth chaining without stepping/stopping between motions.
}
} // namespace

// Executes a straight line drive: distance in inches, optional speed (1-100%), optional timeout
bool drive(double distance_in, double speed_pct, double timeout_s)
{
    if (!std::isfinite(distance_in)) {                             // Verify input distance is a valid number
        show_failure("Auton drive distance invalid");              // Display invalid distance error
        return false;                                              // Abort motion
    }
    if (std::fabs(distance_in) < 1e-6) return true;                // Zero-distance drive completes immediately
    return follow_path(distance_in * kMmPerIn, 0.0, 0.0, speed_pct, timeout_s); // Execute linear path in mm
}

// Executes a relative turn in place: angle in degrees (+90 = turn right 90 deg, -90 = turn left 90 deg)
bool turn(double angle_deg, double speed_pct, double timeout_s)
{
    return turn_by(angle_deg * kDegToRad, speed_pct, timeout_s);   // Convert degrees to radians and execute turn
}

// Executes an absolute turn to a field heading: heading in degrees relative to the field 0-degree reference
bool turn_to(double heading_deg, double speed_pct, double timeout_s)
{
    const double current_heading = imu.rotation(degrees);          // Read current continuous heading in degrees
    const double error_deg = std::remainder(heading_deg - current_heading, 360.0); // Compute shortest-path angle error
    return turn_by(error_deg * kDegToRad, speed_pct, timeout_s);   // Execute turn by shortest-path error
}

// Executes a curved Bezier drive: forward displacement in inches, sideways in inches, end heading in degrees
bool bezier_drive(double forward_in, double right_in, double end_heading_deg, double speed_pct, double timeout_s)
{
    if (!std::isfinite(forward_in) || !std::isfinite(right_in) ||  // Verify all input coordinates are finite numbers
        !std::isfinite(end_heading_deg)) {
        show_failure("Auton bezier input invalid");                // Display invalid input message
        return false;                                              // Abort curve
    }
    return follow_path(forward_in * kMmPerIn, right_in * kMmPerIn, // Convert inputs to mm and radians
                       end_heading_deg * kDegToRad, speed_pct, timeout_s);
}

// Actively stops and holds the drivetrain using motor brake mode
void sdrive()
{
    stop_drive();                                                  // Apply active brake to all 4 motors
}
