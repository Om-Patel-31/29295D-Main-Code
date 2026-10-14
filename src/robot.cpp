#include "vex.h"
#include "config.h"
#include "drive.h"
#include "odometry.h"
#include "tuning.h"

#include <cstdio>

// Hardware integration layer for the V5 Brain, drivetrain, manipulators, UI,
// odometry, autonomous path execution, and driver control. Physical distances
// are converted to mm; headings use rad internally and motor commands use V.
using namespace vex;

// The VS Code VEX project does not generate a Brain instance in vex.h, so the
// application owns the single Brain object used by the screen and SD-card APIs.
vex::brain Brain;
vex::controller Controller = vex::controller(vex::primary);
vex::inertial imu(vex::PORT10);
vex::motor left_front(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor left_rear(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor right_front(vex::PORT9, vex::gearSetting::ratio18_1, true);
vex::motor right_rear(vex::PORT8, vex::gearSetting::ratio18_1, true);
vex::motor lift_left(vex::PORT3, vex::gearSetting::ratio6_1, false);
vex::motor lift_right(vex::PORT4, vex::gearSetting::ratio6_1, true);
vex::motor claw(vex::PORT5, vex::gearSetting::ratio18_1, false);
// Tracking wheel rotation sensors — ports are mock values (see config.h).
// TODO: confirm port wiring before enabling kUseParallelWheelForward in odometry.cpp.
vex::rotation tracking_parallel(config::tracking_parallel_port);
vex::rotation tracking_perp(config::tracking_perp_port);
// AI Vision sensor — port is a mock value (see config.h). TODO: confirm port.
vex::aivision vision_sensor(config::vision_sensor_port);
// Port and reversal values are the configured electrical mapping in this file;
// they describe software polarity, not an assumed physical robot layout.
Odometry g_odometry(config::horizontal_wheel_offset_mm);


namespace {
// This color matches the reference design's gold accent and is the exact team color requested for the UI.
const vex::color kGold(196, 180, 121); // #c4b479
// This color keeps the text on the brain screen readable against the black background in the design reference.
const vex::color kWhite(255, 255, 255);

// This enum tracks which screen the robot is currently showing to the operator.
enum class ScreenMode {
    ModeSelect,       // The first screen where the driver chooses Driver or Autonomous.
    DriverSelector,    // The drive graph selector after Driver is chosen.
    DriverSelected,    // The selected driver graph after the selector is confirmed.
    AutonSelected      // The autonomous selection path after the mode is chosen.
};

// This enum keeps track of which autonomous path the operator selected.
enum class AutonSelection {
    None,             // No autonomous option has been chosen yet.
    LeftSide,         // The left-side autonomous routine is selected.
    Skills,           // The skills autonomous routine is selected.
    RightSide         // The right-side autonomous routine is selected.
};

// This global variable stores the current screen state for the brain UI loop.
ScreenMode g_screen_mode = ScreenMode::ModeSelect;
// This global variable stores the autonomous selection so it can be used later in autonomous().
AutonSelection g_selected_auton = AutonSelection::None;
DriverGraph g_selected_driver_graph = DriverGraph::Cubic;
bool g_imu_calibration_failed = false;

// AprilTag detection rate-limiting constants. takeSnapshot is called at most
// every kVisionPollMs ms to avoid blocking the 10 ms driver loop.
static const int      kTagTriggerWidthPx  = 80; // TODO: tune on field (placeholder)
static const int      kTagTriggerHeightPx = 80; // TODO: tune on field (placeholder)
static const uint32_t kVisionPollMs       = 100;

void StopDriveMotors()
{
    // Every autonomous failure, timeout, and completion funnels through this
    // helper so no path exit intentionally leaves drive voltage applied.
    left_front.stop();
    left_rear.stop();
    right_front.stop();
    right_rear.stop();
}

OdometryMeasurement ReadOdometryMeasurement()
{
    static const double kDegToRad = 3.141592653589793 / 180.0;
    // imu.rotation() gives unbounded signed angle (does not wrap at 360 deg),
    // which is required so heading deltas remain continuous across 0/360.
    return {
        average_side(left_front, left_rear),
        average_side(right_front, right_rear),
        tracking_parallel.position(degrees) *
            (config::tracking_wheel_circumference_mm / 360.0),
        tracking_perp.position(degrees) *
            (config::tracking_wheel_circumference_mm / 360.0),
        imu.rotation(degrees) * kDegToRad,
        // gyroRate(zaxis, dps) returns deg/s; convert to rad/s.
        imu.gyroRate(zaxis, dps) * kDegToRad
    };
}


void ShowAutonomousMessage(const char* message)
{
    StopDriveMotors();
    Brain.Screen.clearScreen();
    Brain.Screen.setPenColor(kGold);
    Brain.Screen.printAt(20, 120, message);
}

void run_path(double forward_distance, double turn_distance, double heading)
{
    // A path run has four phases: construct geometry, reset odometry, start
    // both feedback loops, then update at 100 Hz until completion or timeout.
    BezierPath path;
    const double pi = 3.141592653589793;
    const double end_heading = heading * pi / 180.0;
    if (!path.Configure(forward_distance, turn_distance, end_heading,
                        config::min_turn_radius_mm)) {
        // Invalid geometry is a safe exit and stops all drive motors.
        ShowAutonomousMessage("Auton path invalid");
        return;
    }

    const OdometryMeasurement initial = ReadOdometryMeasurement();
    g_odometry.Reset(initial, initial.inertial_heading);
    const Pose2 measured = g_odometry.pose();
    DriveController drive(config::longitudinal_wc, config::longitudinal_b0,
                          config::heading_wc, config::heading_b0);
    if (!drive.StartPath(path, measured, config::v_max_mm_s,
                         config::a_max_mm_s2, config::j_max_mm_s3)) {
        // Invalid gains or profile limits are reported before motor commands.
        ShowAutonomousMessage("Auton controller invalid");
        return;
    }

    // The loop bound is autonomous_timeout_s seconds at 100 Hz, with each
    // iteration sleeping 10 ms. The controller itself has no stall timeout.
    const int max_cycles = static_cast<int>(
        config::autonomous_timeout_s * config::control_hz);
    for (int cycle = 0; cycle < max_cycles && drive.active(); ++cycle) {
        const Pose2 pose = g_odometry.Update(
            1.0 / config::control_hz, ReadOdometryMeasurement());
        const double supply = Brain.Battery.voltage(vex::volt);
        const DriveVoltages output = drive.Update(
            1.0 / config::control_hz, pose, supply);
        if (drive.faulted()) {
            // A LADRC validity fault stops motors before returning to autonomous.
            ShowAutonomousMessage("Auton controller fault");
            return;
        }
        left_front.spin(forward, output.left, volt);
        left_rear.spin(forward, output.left, volt);
        right_front.spin(forward, output.right, volt);
        right_rear.spin(forward, output.right, volt);
        wait(10, msec);
    }

    StopDriveMotors();
    if (drive.active()) {
        ShowAutonomousMessage("Auton timeout");
    }
}

// This structure describes one UI button so we can handle touch events and draw consistent controls.
struct ScreenButton {
    int x;              // The left-most x coordinate of the button on the screen.
    int y;              // The top-most y coordinate of the button on the screen.
    int width;          // The width of the button in pixels.
    int height;         // The height of the button in pixels.
    const char* label;  // The text that appears in the button.
};

// This helper checks if a touch point falls inside a button's rectangle.
bool IsPointInsideButton(const ScreenButton& button, int x, int y) {
    // The x coordinate must be at or to the right of the left edge and at or to the left of the right edge.
    return x >= button.x && x <= button.x + button.width &&
           // The y coordinate must be at or below the top edge and at or above the bottom edge.
           y >= button.y && y <= button.y + button.height;
}

// This function draws one large button using the same styling as the reference design.
void DrawButton(const ScreenButton& button, bool selected) {
    // The button fill is always gold in the reference mockup, but this selected flag is kept so the code can still show a highlight if needed later.
    const vex::color fill_color = selected ? kGold : kGold;
    // The text on the buttons is white to match the black-and-gold design reference.
    const vex::color text_color = kWhite;

    // This sets the fill color used by the rectangle drawing routine.
    Brain.Screen.setFillColor(fill_color);
    // This sets the text color used by printAt() when drawing labels.
    Brain.Screen.setPenColor(text_color);
    // This draws the rounded rectangle background that matches the gold panel style in the provided reference.
    Brain.Screen.drawRectangle(button.x, button.y, button.width, button.height, fill_color);
    // This draws the label text centered within the button area to match the mockup layout when a label is provided.
    if (button.label != nullptr && button.label[0] != '\0') {
        Brain.Screen.printAt(button.x + 12, button.y + (button.height / 2) + 6, button.label);
    }
}

// This function draws the final logo screen after the user has chosen a mode and, if needed, an autonomous routine.
void DrawSelectedLogoOnly(ScreenMode mode, AutonSelection auton) {
    // Clear the screen before showing the final image so the background is completely black and the logo is the focus.
    Brain.Screen.clearScreen();

    // According to the V5 API, the SD card must be inserted and the file must exist in the root directory before Image operations can succeed.
    const bool sd_inserted = Brain.SDcard.isInserted();
    const bool logo_exists = sd_inserted && Brain.SDcard.exists("29295D_Logo.png");

    if (!sd_inserted) {
        Brain.Screen.setPenColor(kGold);
        Brain.Screen.printAt(20, 100, "Insert SD card");
        return;
    }

    if (!logo_exists) {
        Brain.Screen.setPenColor(kGold);
        Brain.Screen.printAt(20, 100, "29295D_Logo.png");
        Brain.Screen.printAt(20, 135, "not found");
        return;
    }

    // Draw the logo file from the SD card root directory using the V5 Brain Screen API.
    Brain.Screen.drawImageFromFile("29295D_Logo.png", 0, 0);
    // Set the text color to gold to match the branding and reference layout.
    Brain.Screen.setPenColor(kGold);

    // If the operator picked Driver Control, show a driver label over the logo.
    if (mode == ScreenMode::DriverSelected) {
        // Print the driver label near the lower portion of the display for a clean final state.
        Brain.Screen.printAt(20, 235, "Driver");
    } else {
        // Default to Right Side text for a generic auton label if the autonomous selection is not explicitly set.
        const char* auton_name = "Right Side";
        // If the operator selected the left-side routine, update the label to match their choice.
        if (auton == AutonSelection::LeftSide) {
            auton_name = "Left Side";
        } else if (auton == AutonSelection::Skills) {
            // If Skills is chosen, use the exact wording shown in the provided design reference.
            auton_name = "Skills";
        }
        // Display the selected autonomous routine name over the logo.
        Brain.Screen.printAt(20, 235, false, "Auton: %s", auton_name);
    }
}

void DrawDriverSelector(ScreenMode& mode, DriverGraph& graph) {
    Brain.Screen.clearScreen();
    Brain.Screen.setPenColor(kWhite);
    Brain.Screen.printAt(105, 25, "29295D Drive Selector");

    const ScreenButton cubic{25, 55, 125, 55, ""};
    const ScreenButton quintic{177, 55, 125, 55, ""};
    const ScreenButton quintic_bounds{330, 55, 140, 55, ""};
    const ScreenButton trig_quintic{25, 145, 165, 55, ""};
    const ScreenButton trig_bounds{290, 145, 180, 55, ""};

    DrawButton(cubic, false);
    DrawButton(quintic, false);
    DrawButton(quintic_bounds, false);
    DrawButton(trig_quintic, false);
    DrawButton(trig_bounds, false);

    Brain.Screen.setPenColor(kWhite);
    Brain.Screen.printAt(57, 89, "Cubic");
    Brain.Screen.printAt(203, 89, "Quintic");
    Brain.Screen.printAt(340, 76, "Quintic with");
    Brain.Screen.printAt(340, 98, "Trig. bounds");
    Brain.Screen.printAt(38, 177, "Trigonometric");
    Brain.Screen.printAt(65, 198, "Quintic");
    Brain.Screen.printAt(302, 167, "Trigonometric");
    Brain.Screen.printAt(318, 188, "with quintic");
    Brain.Screen.printAt(350, 209, "bounds");

    if (Brain.Screen.pressing()) {
        const int x = Brain.Screen.xPosition();
        const int y = Brain.Screen.yPosition();
        if (IsPointInsideButton(cubic, x, y)) {
            graph = DriverGraph::Cubic;
        } else if (IsPointInsideButton(quintic, x, y)) {
            graph = DriverGraph::Quintic;
        } else if (IsPointInsideButton(quintic_bounds, x, y)) {
            graph = DriverGraph::QuinticTrigonometricBounds;
        } else if (IsPointInsideButton(trig_quintic, x, y)) {
            graph = DriverGraph::TrigonometricQuintic;
        } else if (IsPointInsideButton(trig_bounds, x, y)) {
            graph = DriverGraph::TrigonometricQuinticBounds;
        } else {
            return;
        }
        mode = ScreenMode::DriverSelected;
    }
}

// This function renders the first selection screen showing Driver Control and Autonomous Control buttons.
void DrawModeSelector(ScreenMode& mode, AutonSelection& auton) {
    // Clear any previous screen so the first selection screen is always fresh and consistent.
    Brain.Screen.clearScreen();
    // Set the title color to white to match the high-contrast look in the reference image.
    Brain.Screen.setPenColor(kWhite);
    // Display the exact main title from the design reference in the top-center area.
    Brain.Screen.printAt(60, 30, "29295D Program Selector");

    // Define the driver control button sized and placed to match the provided mockup layout.
    const ScreenButton driver_button{80, 65, 320, 60, "Driver Control"};
    // Define the autonomous control button positioned underneath the driver option to match the reference.
    const ScreenButton auton_button{80, 145, 320, 60, "Autonomous Control"};

    // Draw both selection buttons using the gold panel styling from the reference.
    DrawButton(driver_button, false);
    DrawButton(auton_button, false);

    // Check if the user is touching the brain screen, which indicates a selection press.
    if (Brain.Screen.pressing()) {
        // Read the current touch coordinates so we can determine which button was pressed.
        const int touch_x = Brain.Screen.xPosition();
        const int touch_y = Brain.Screen.yPosition();

        // If the touch is inside the driver button, enter the driver-selected flow and clear any previous auton choice.
        if (IsPointInsideButton(driver_button, touch_x, touch_y)) {
            mode = ScreenMode::DriverSelector;
            auton = AutonSelection::None;
        } else if (IsPointInsideButton(auton_button, touch_x, touch_y)) {
            // If the touch is inside the auton button, open the autonomous chooser and reset the current auton choice.
            mode = ScreenMode::AutonSelected;
            auton = AutonSelection::None;
        }
    }
}

// This function renders the second stage screen where the operator chooses Left Side, Skills, or Right Side.
void DrawAutonSelector(ScreenMode& mode, AutonSelection& auton) {
    // Clear the previous screen to show the auton selector without any leftover UI.
    Brain.Screen.clearScreen();
    // Use white text to match the front-end mockup and keep the title readable on the black background.
    Brain.Screen.setPenColor(kWhite);
    // Display the title shown in the supplied design reference for the auton screen.
    Brain.Screen.printAt(60, 30, "29295D Auton Selector");

    // These three options are actual touch buttons, not just static text, so they behave like the other mode-selection controls.
    const ScreenButton left_button{20, 75, 140, 110, ""};
    const ScreenButton skills_button{170, 75, 140, 110, ""};
    const ScreenButton right_button{320, 75, 140, 110, ""};

    // Draw all three auton buttons using the same gold panel styling as the rest of the selector design.
    DrawButton(left_button, false);
    DrawButton(skills_button, false);
    DrawButton(right_button, false);

    // Since the reference uses stacked text for the side buttons, we place the words inside each button to match the mockup exactly.
    Brain.Screen.setPenColor(kWhite);
    Brain.Screen.printAt(left_button.x + 28, left_button.y + 42, "Left");
    Brain.Screen.printAt(left_button.x + 22, left_button.y + 75, "Side");
    Brain.Screen.printAt(skills_button.x + 30, skills_button.y + 60, "Skills");
    Brain.Screen.printAt(right_button.x + 22, right_button.y + 42, "Right");
    Brain.Screen.printAt(right_button.x + 28, right_button.y + 75, "Side");

    // Check whether the user pressed the screen to choose an auton path.
    if (Brain.Screen.pressing()) {
        // Read the touch location so the correct choice can be matched to the right button rectangle.
        const int touch_x = Brain.Screen.xPosition();
        const int touch_y = Brain.Screen.yPosition();

        // If the touch is inside the left-side button, store the left-side autonomous selection.
        if (IsPointInsideButton(left_button, touch_x, touch_y)) {
            auton = AutonSelection::LeftSide;
            mode = ScreenMode::AutonSelected;
        } else if (IsPointInsideButton(skills_button, touch_x, touch_y)) {
            // If the touch is inside the skills button, store the skills autonomous selection.
            auton = AutonSelection::Skills;
            mode = ScreenMode::AutonSelected;
        } else if (IsPointInsideButton(right_button, touch_x, touch_y)) {
            // If the touch is inside the right-side button, store the right-side autonomous selection.
            auton = AutonSelection::RightSide;
            mode = ScreenMode::AutonSelected;
        }
    }
}
} // namespace


void pre_auton()
{
    // IMU calibration is limited to 5.0 s / 0.02 s = 250 cycles. The wait is
    // 20 ms per cycle, so the actual current loop bound is approximately 5.0 s.
    g_imu_calibration_failed = false;
    imu.calibrate();
    const int max_cycles = static_cast<int>(
        config::inertial_calibration_timeout_s /
        config::inertial_calibration_period_s);
    for (int cycle = 0; imu.isCalibrating() && cycle < max_cycles; ++cycle) {
        wait(20, msec);
    }
    if (imu.isCalibrating()) {
        g_imu_calibration_failed = true;
        ShowAutonomousMessage("IMU calibration timeout");
    }

    // Reset all position sensors so odometry starts from a known zero state.
    // resetRotation() resets the unbounded rotation() counter used by
    // ReadOdometryMeasurement(); resetHeading() would only affect heading().
    tracking_parallel.resetPosition();
    tracking_perp.resetPosition();
    imu.resetRotation();
    const OdometryMeasurement init = ReadOdometryMeasurement();
    g_odometry.Reset(init, 0.0);

    // Enable AprilTag detection on the AI Vision sensor.
    vision_sensor.tagDetection(true);
    g_vision_timer.reset();
}


void autonomous()
{
    // Autonomous is gated before any path construction or motor command while
    // either b0 calibration value is non-positive or IMU calibration failed.
    if (g_imu_calibration_failed ||
        config::longitudinal_b0 <= 0.0 || config::heading_b0 <= 0.0) {
        Brain.Screen.clearScreen();
        Brain.Screen.setPenColor(kGold);
        Brain.Screen.printAt(20, 120, g_imu_calibration_failed
            ? "IMU calibration failed" : "Calibrate b0 first");
        StopDriveMotors();
        return;
    }

    switch (g_selected_auton) {
        case AutonSelection::RightSide:
            run_path(600.0, 300.0, 0.0);
            run_path(400.0, -200.0, 300.0);
            break;
        case AutonSelection::LeftSide:
            run_path(-600.0, 300.0, 0.0);
            run_path(-400.0, -200.0, 300.0);
            break;
        case AutonSelection::Skills:
            run_path(600.0, 0.0, 0.0);
            break;
        default:
            run_path(600.0, 300.0, 0.0);
            run_path(400.0, -200.0, 300.0);
            break;
    }
}

static void run_lift_and_claw()
{
    const int lift_power =
        (Controller.ButtonR1.pressing() ? 100 : 0) -
        (Controller.ButtonR2.pressing() ? 100 : 0);
    const int claw_power =
        (Controller.ButtonL1.pressing() ? 100 : 0) -
        (Controller.ButtonL2.pressing() ? 100 : 0);

    if (lift_power == 0) {
        lift_left.stop();
        lift_right.stop();
    } else {
        lift_left.spin(forward, lift_power, percent);
        lift_right.spin(forward, lift_power, percent);
    }

    if (claw_power == 0) {
        claw.stop();
    } else {
        claw.spin(forward, claw_power, percent);
    }
}

void usercontrol()
{
    // Driver control intentionally bypasses LADRC and b0 calibration. Axis 3
    // supplies forward/reverse power and Axis 1 supplies turn power; the
    // signed sums form arcade left/right commands in V.
    const DriverGains& gains = driver_gains(g_selected_driver_graph);
    while (true) {
        const double forward_power =
            Controller.Axis3.position(percent) * gains.acceleration_gain;
        const double turn_power =
            Controller.Axis1.position(percent) * gains.turning_gain;

        left_front.spin(forward, forward_power + turn_power, volt);
        left_rear.spin(forward, forward_power + turn_power, volt);
        right_front.spin(forward, forward_power - turn_power, volt);
        right_rear.spin(forward, forward_power - turn_power, volt);
        run_lift_and_claw();
        wait(10, msec);
    }
}

void run_screen_selector()
{
    // The selector is a 50 ms UI task. It records mode choices, then displays
    // the SD-card logo after the final driver or autonomous choice.
    while (true) {
        if (g_screen_mode == ScreenMode::ModeSelect) {
            DrawModeSelector(g_screen_mode, g_selected_auton);
        } else if (g_screen_mode == ScreenMode::DriverSelector) {
            DrawDriverSelector(g_screen_mode, g_selected_driver_graph);
        } else if (g_screen_mode == ScreenMode::AutonSelected && g_selected_auton == AutonSelection::None) {
            DrawAutonSelector(g_screen_mode, g_selected_auton);
        } else {
            DrawSelectedLogoOnly(g_screen_mode, g_selected_auton);
        }

        wait(50, msec);
    }
}
