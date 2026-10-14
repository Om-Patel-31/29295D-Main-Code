# Drivetrain tuning

## Calibration

For each control channel, apply a safe voltage step `u` and record elapsed time
and measured velocity until the response settles. Fit

`v(t) = v_ss * (1 - exp(-t / tau))`

with `FitStepResponse`. Then calculate:

`b0 = v_ss / (tau * u)`

The longitudinal input `u` is the commanded longitudinal voltage. For heading,
`u` is the heading-channel voltage `U_turn`. The mixer applies
`V_L = U_long - U_turn` and `V_R = U_long + U_turn`, so the physical wheel
difference is `2 * U_turn`; this factor is accounted for by measuring and
fitting the heading response against the heading-channel input, not one wheel's
voltage.

Store the measured positive gains in `include/config.h`. The autonomous adapter
refuses to start while either value is still zero.

## Bandwidth

For a desired closed-loop settling time `T_s`, start with
`w_c = 4 / T_s` in rad/s. The observer defaults to `w_o = 4 * w_c`.
Raise `w_o` only when disturbance rejection is too slow and sensor noise remains
acceptable. Lower it when noise is amplified.

The S-curve startup check requires positive finite `b0`, headroom
`a_max / (b0 * 12 V) <= 0.7`, and the heuristic `j_max / a_max <= w_c`.
