# Implementation Plan — 29295D-Main-Code

**Team:** 29295D VEX V5RC Override  
**SDK:** V5_20240802_15_00_00  
**Date drafted:** 2026-10-13  

---

## Phase 0 Findings: API Verification

All member names below were confirmed against the installed SDK headers at  
`C:\Users\om31d\AppData\Roaming\Code\User\globalStorage\vexrobotics.vexcode\sdk\cpp\V5\V5_20240802_15_00_00\vexv5\include\`.

| Needed name | Source header | Status |
|---|---|---|
| `vex::aivision` | `vex_aivision.h` | ✅ exists — class name is `aivision` (not `ai_vision`) |
| `aivision::takeSnapshot(const tagdesc&, int32_t)` | `vex_aivision.h` | ✅ exists |
| `aivision::tagDetection(bool)` | `vex_aivision.h` | ✅ exists |
| `aivision::objectCount` | `vex_aivision.h` | ✅ public data member |
| `aivision::objects[i]` | `vex_aivision.h` | ✅ `safearray<object,24>` |
| `aivision::object::id`, `.width`, `.height`, `.type` | `vex_aivision.h` | ✅ read-only references |
| `aivision::ALL_TAGS` | `vex_aivision.h` | ✅ `static const tagdesc` |
| `vex::rotation` | `vex_rotation.h` | ✅ exists |
| `rotation::position(rotationUnits)` | `vex_rotation.h` | ✅ — takes `rotationUnits` (use `degrees`) |
| `rotation::resetPosition()` | `vex_rotation.h` | ✅ |
| `vex::inertial::rotation(rotationUnits)` | `vex_imu.h` | ✅ — unbounded signed angle |
| `vex::inertial::heading(rotationUnits)` | `vex_imu.h` | ✅ — 0–360 wrapped |
| `vex::inertial::gyroRate(axisType, velocityUnits)` | `vex_imu.h` | ✅ — use `zaxis`, `dps` |
| `vex::inertial::resetHeading()` | `vex_imu.h` | ✅ |
| `vex::inertial::resetRotation()` | `vex_imu.h` | ✅ |
| `vex::motor::position(rotationUnits)` | `vex_motor.h` | ✅ |
| `vex::motor::stop(brakeType)` | `vex_motor.h` | ✅ — use `hold` |
| `vex::timer` | `vex_timer.h` | ✅ |
| `timer::time()` | `vex_timer.h` | ✅ returns `uint32_t` ms |
| `timer::reset()` / `timer::clear()` | `vex_timer.h` | ✅ both exist |
| `vex::zaxis` | `vex_global.h` | ✅ — `axisType` constant |
| `vex::dps` | `vex_global.h` | ✅ — `velocityUnits` constant |
| `vex::degrees` / `vex::deg` | `vex_global.h` | ✅ |

> **NOTE — `gyroRate` unit:** The public overload `gyroRate(axisType, velocityUnits)` takes `velocityUnits`.
> `vex::dps` is declared as `const velocityUnits` in `vex_global.h`. ✅  
> Converting dps → rad/s: multiply by `π / 180.0`.

> **NOTE — `include/robot.h` does not exist.** All hardware globals are declared directly in `src/robot.cpp`
> with `extern` declarations provided by `main.cpp` forward-declaring only the three callback
> functions. No action needed; the plan notes this fact.

---

## Files to Change — by Phase

### Phase 1 — Odometry fixes

#### `include/odometry.h`
- Add `double horizontal_offset_mm_;` as a private member **after** `heading_bias_` (line 45) so
  it matches the constructor-initializer order.
- Current line 45: `double heading_bias_, velocity_, turn_rate_;` → split and insert the new member.

#### `src/odometry.cpp`
1. **Constructor signature**: rename the unnamed `double` parameter to `horizontal_wheel_offset_mm`
   and initialize `horizontal_offset_mm_` from it. The current constructor discards its argument.
2. **`Update()` — forward distance**: add file-local
   `static const bool kUseParallelWheelForward = true;`
   When `true`, use `m.vertical_mm - previous_.vertical_mm` as forward. When `false`, keep `0.5*(dl+dr)`.
3. **`Update()` — sideways correction**: replace `heading_bias_ * dtheta` with
   `horizontal_offset_mm_ * dtheta`.  
   Full expression: `sideways = dh - horizontal_offset_mm_ * dtheta`  
   where `dh = m.horizontal_mm - previous_.horizontal_mm` and `dtheta` is already in radians.
4. All existing non-finite checks and the `dt > 0` guard are kept verbatim.

> **CAUTION:** The current constructor body `Odometry::Odometry(double)` silently ignores its
> argument. This is the primary correctness bug in odometry.

#### `src/robot.cpp` — `ReadOdometryMeasurement()`
Replace the zero-filled and `imu.heading()`-based fields with:
- `imu.rotation(degrees) * (M_PI / 180.0)` for `inertial_heading`
- `tracking_parallel.position(degrees) * (config::tracking_wheel_circumference_mm / 360.0)` for `vertical_mm`
- `tracking_perp.position(degrees)    * (config::tracking_wheel_circumference_mm / 360.0)` for `horizontal_mm`
- `imu.gyroRate(zaxis, dps) * (M_PI / 180.0)` for `inertial_rate`

#### `include/config.h`
Add inside `namespace config`:
- `tracking_wheel_circumference_mm = 50.8 * M_PI` (2" wheel)
- `tracking_parallel_port = 6` — **ASSUMPTION** (TODO: confirm wiring)
- `tracking_perp_port     = 7` — **ASSUMPTION** (TODO: confirm wiring)
- `vision_sensor_port     = 11` — **ASSUMPTION** (TODO: confirm wiring)
- Keep `horizontal_wheel_offset_mm` at 0.0 with updated TODO comment.

---

### Phase 2 — Tracking-wheel hardware

#### `src/robot.cpp`
- Declare at file scope after existing motor declarations:
  ```cpp
  vex::rotation tracking_parallel(config::tracking_parallel_port);
  vex::rotation tracking_perp(config::tracking_perp_port);
  vex::aivision vision_sensor(config::vision_sensor_port);
  ```
- In `pre_auton()`, **after** the calibration wait loop, add:
  ```cpp
  tracking_parallel.resetPosition();
  tracking_perp.resetPosition();
  imu.resetRotation();
  const OdometryMeasurement init = ReadOdometryMeasurement();
  g_odometry.Reset(init, 0.0);
  ```

> **NOTE:** `imu.resetRotation()` resets the unbounded `rotation()` value we now use.
> `imu.resetHeading()` only resets the 0–360 `heading()` value, which we no longer use for odometry.

---

### Phase 3 — Lift PID (inches)

#### `include/config.h`
Add:
```cpp
// Lift geometry — TODO: measure both values on the robot.
static const double lift_total_reduction      = 1.0; // TODO: output_rev / motor_rev (all stages)
static const double lift_sprocket_diameter_in = 2.0; // TODO: pitch diameter in inches
```

#### `src/robot.cpp`
- Add `struct LiftPID { double kp, ki, kd, integral, previous_error, integral_max; };`
- Add a helper `double lift_inches()` that reads `lift_left.position(degrees)`, converts via
  `lift_total_reduction` and sprocket circumference, and returns inches.
- Add `void lift_move_to_inches(double target_in, int timeout_ms)`:
  - PID loop at 10 ms intervals with integral clamp and ±12 V output clamp.
  - Exits on timeout or |error| < threshold.
  - Calls `lift_left.stop(hold); lift_right.stop(hold);` on exit.
- Add `bool g_lift_macro_running = false;`.
- Modify `run_lift_and_claw()`: R1/R2 press sets `g_lift_macro_running = false` (cancels macro)
  and switches to manual voltage.

> **ASSUMPTION:** Positive voltage on `lift_left` (PORT3, not reversed) moves the lift UP. Verify on hardware.

---

### Phase 4 — Claw timed sequence

#### `include/config.h`
Add:
```cpp
// Claw timed open-loop constants.
// ASSUMPTION: positive power closes the claw — verify on hardware.
static const double claw_close_power_pct =  80.0; // TODO: tune
static const int    claw_close_time_ms   = 600;   // TODO: tune
static const double claw_open_power_pct  = -80.0; // TODO: tune
static const int    claw_open_time_ms    = 500;   // TODO: tune
```

#### `src/robot.cpp`
Add:
```cpp
void run_claw_timed(double power_pct, int duration_ms) {
    claw.spin(forward, power_pct, percent);
    wait(duration_ms, msec);
    claw.stop();
}
```

---

### Phase 5 — AprilTag detection (test mode)

#### `src/robot.cpp`
- Enable tag detection in `pre_auton()`: `vision_sensor.tagDetection(true);`
- Add file-scope (anonymous namespace) constants:
  ```cpp
  static const int      kTagTriggerWidthPx  = 80; // TODO: tune on field
  static const int      kTagTriggerHeightPx = 80; // TODO: tune on field
  static const uint32_t kVisionPollMs       = 100;
  ```
- Add a file-scope `vex::timer g_vision_timer;`; reset it in `pre_auton()`.
- Add `void poll_vision()`:
  - Returns immediately if `g_vision_timer.time() < kVisionPollMs`.
  - Calls `vision_sensor.takeSnapshot(aivision::ALL_TAGS)`.
  - Resets `g_vision_timer`.
  - Iterates `vision_sensor.objectCount`, prints tag `id`, `width`, `height` to Brain screen
    when `width >= kTagTriggerWidthPx && height >= kTagTriggerHeightPx`.
- Call `poll_vision()` inside `usercontrol()`'s `while(true)` loop.

> **NOTE:** `takeSnapshot` is never called inside the tight 10 ms drive iteration; the timer guard
> ensures ≥100 ms between calls even when `poll_vision` is called every loop.

---

### Phase 6 — Scoring macro

#### `include/config.h`
Add:
```cpp
// Lift target heights for scoring — TODO: measure on field.
static const double lift_height_pins_in  = 8.0;   // TODO
static const double lift_height_cups_in  = 5.0;   // TODO
static const double macro_cooldown_ms    = 2000.0; // ms between macro triggers
```

#### `src/robot.cpp`
- Add `bool g_macro_running = false;` and `vex::timer g_macro_cooldown;`.
- Implement `score_pins()`, `score_cups()`, `score_both()`:
  - Set `g_macro_running = true`.
  - Each step: `lift_move_to_inches(...)` then `run_claw_timed(...)`.
  - After each step: `if (!g_macro_running) return;`
  - Clear `g_macro_running` at end.
  - On normal completion: `g_macro_cooldown.reset();`
- In `poll_vision()`: trigger `score_both()` when a qualifying tag is seen and
  `!g_macro_running && g_macro_cooldown.time() > config::macro_cooldown_ms`.
- Manual R1/R2 in `run_lift_and_claw()` already clears `g_macro_running`.

> **OPEN QUESTION (Q8):** Macro steps call blocking `lift_move_to_inches()`, which will stall
> the driver-control loop. Recommend running macros in a separate `vex::task`. Final approach
> requires approval before Phase 6 begins.

---

## Open Questions

| # | Question | Blocks |
|---|---|---|
| Q1 | **Tracking wheel ports:** Ports 6 and 7 assumed. Confirm Smart port wiring. | Phase 2 |
| Q2 | **Vision sensor port:** Port 11 assumed. Confirm. | Phase 2 |
| Q3 | **Lift gear ratio (`lift_total_reduction`):** Full mechanical reduction unknown. Must be measured. | Phase 3 |
| Q4 | **Lift sprocket diameter:** Placeholder 2.0 in. Must be measured. | Phase 3 |
| Q5 | **Lift motor polarity:** Is positive voltage on `lift_left` (PORT3) the UP direction? | Phase 3 |
| Q6 | **Claw polarity:** Is positive power the CLOSE direction? | Phase 4 |
| Q7 | **`horizontal_wheel_offset_mm`:** 0.0 placeholder. Must be measured (perpendicular distance, rotation center → wheel contact). | Phase 1 |
| Q8 | **Macro concurrency:** Blocking loop or separate VEX task? | Phase 6 |
| Q9 | **`kUseParallelWheelForward` default:** If tracking wheel not yet installed, set to `false` to fall back to drive encoders. | Phase 1 |
| Q10 | **`imu.resetRotation()` vs `imu.resetHeading()`:** We use `resetRotation()` because we read `rotation()`. Confirmed in header. | Phase 2 |

---

## Build Target

```
make -C c:\Users\om31d\Documents\vex-vscode-projects\29295D-Main-Code
```

---

## Files NOT Modified (any phase)

`src/main.cpp`, `src/tuning.cpp`, `include/tuning.h`, `src/bezier.cpp`, `src/drive.cpp`,
`src/ladrc2.cpp`, `src/scurve.cpp`, `include/bezier.h`, `include/drive.h`, `include/ladrc2.h`,
`include/scurve.h`, `vex/mkenv.mk`, `vex/mkrules.mk`, `makefile`.

---

*Awaiting approval before Phase 1 begins.*
