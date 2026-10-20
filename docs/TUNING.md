# Drivetrain tuning

## Calibration

All user-facing configuration, tuning, and autonomous calls use **inches** for distance and **degrees** for angles.

For each control channel, apply a safe voltage step `u` and record elapsed time
and measured velocity until the response settles. Fit

`v(t) = v_ss * (1 - exp(-t / tau))`

with `FitStepResponse`. Then calculate:

`b0 = v_ss / (tau * u)`

Automated routines are provided in `src/tuning.cpp`:
- `tune_drive_b0(voltage, duration)` measures response in **in/s** and outputs `drive_b0` in **in/s^2/V**.
- `tune_turn_b0(voltage, duration)` measures gyro response in **deg/s** and outputs `turn_b0` in **deg/s^2/V**.

The longitudinal input `u` is the commanded longitudinal voltage. For heading,
`u` is the heading-channel voltage `U_turn`. The mixer applies
`V_L = U_long + U_turn` and `V_R = U_long - U_turn`, so a positive `U_turn` turns clockwise (matching V5 inertial rotation).

Store the measured positive gains in `include/config.h`:
- `drive_b0_in_s2_per_v`
- `turn_b0_deg_s2_per_v`

## Autonomous Motion Commands

Autonomous routines live in `src/autons.cpp`. All distances are in **inches** and angles in **degrees**:
- `drive(distance_in, speed_pct, timeout_s)` — straight drive forward or backward (negative distance).
- `turn(angle_deg, speed_pct, timeout_s)` — relative turn (+deg is clockwise / right).
- `turn_to(heading_deg, speed_pct, timeout_s)` — absolute turn to starting-heading frame (takes shortest path).
- `bezier_drive(forward_in, right_in, end_heading_deg, speed_pct, timeout_s)` — smooth curved S-curve trajectory.

All commands block until settled and return `false` on fault/timeout.

## Bandwidth & Settling Time

In `include/config.h`, set desired loop settling times directly:
- `drive_response_time_s` (e.g. 1.0 s)
- `turn_response_time_s` (e.g. 1.0 s)

Internal closed-loop bandwidths are derived automatically: `w_c = 4 / response_time`.
The S-curve startup check requires positive finite `b0`, headroom
`a_max / (b0 * 12 V) <= 0.7`, and the heuristic `j_max / a_max <= w_c`.
