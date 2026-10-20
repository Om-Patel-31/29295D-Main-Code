# IEEE-Style Manuscript Draft — Results Required Before Submission

> **Status:** This is a technically grounded manuscript draft, not a publishable empirical paper yet. The repository contains a controller implementation and host tests, but no documented physical-robot experimental dataset, verified calibration, or comparative results. Bracketed `TODO` fields must be completed from real trials. Do not submit with placeholders or infer performance claims from source code.

## Title

**Bandwidth-Parameterized LADRC with Quintic Time Scaling for Differential-Drive Path Tracking: An Implementation and Evaluation Framework**

## Abstract

This paper describes an implementation of bandwidth-parameterized linear active disturbance rejection control (LADRC) for longitudinal and heading motion on a VEX V5 differential-drive robot. The implementation combines a third-order extended-state observer (ESO), a fifth-degree point-to-point time-scaling profile, sampled cubic Bézier path geometry, proportional cross-track heading correction, and common-mode wheel-voltage scaling. We document the control equations and software architecture and specify an experimental protocol for assessing tracking under repeatable conditions. The controller gains in the repository are configuration values, not identified physical parameters. **No physical-robot results or controlled baseline comparison are available for this draft.** Consequently, no claim of improved accuracy, disturbance rejection, stability, or reduced execution time is made. Before submission, the method must be evaluated on hardware with calibrated independent ground truth, identified plant gains, repeated trials, and a documented comparator. `TODO: Insert measured results and uncertainty only after data collection.`

**Index Terms—** active disturbance rejection control, differential-drive robots, mobile robot path following, trajectory generation, experimental evaluation.

## I. Introduction

Differential-drive robots must coordinate translational and angular motion while operating with actuator saturation, changing battery voltage, uncertain friction, and imperfect state estimates. A controller that tracks a smooth reference in software does not, by itself, establish accurate physical motion: actuator limits, sensor calibration, wheel slip, odometry bias, and the test surface all affect the observed trajectory.

Active disturbance rejection control represents unmodeled plant effects as a lumped disturbance and estimates that disturbance using an extended-state observer. Linear ADRC (LADRC) can be parameterized through controller and observer bandwidths [1], [2]. Separately, a fifth-degree polynomial satisfying endpoint position, velocity, and acceleration constraints gives a smooth point-to-point time scaling; under the stated boundary conditions it is the standard minimum-integrated-squared-jerk solution [3]. Neither method guarantees a particular robot-level performance without validated modeling and experiments.

This work documents the implementation in the repository and defines a reproducible evaluation path. The software has separate LADRC channels for path progress and robot heading, a scalar quintic time law, and sampled cubic Bézier geometry. The present contribution is an implementation description and testable experimental design—not a validated performance claim. In particular, the current source does not establish a stability proof for the complete sampled, saturated, coupled robot, and this draft does not present physical experimental results.

## II. Platform and Problem Formulation

The target is a VEX V5 differential-drive robot. The software obtains inertial heading and wheel/tracking-wheel encoder readings, estimates pose through odometry, and commands left and right motor groups in voltage units. Exact robot mass, wheel characteristics, sensor calibration, firmware/SDK versions, battery condition, and floor properties are not specified here and must be recorded for each study (`TODO`). Hardware specifications should be cited to the manufacturer only if relevant to the final analysis; this draft makes no processor-performance claim.

For each controlled scalar coordinate, the design uses the nominal second-order model
\[
  \ddot y(t)=f(t)+b_0 u(t),
\]
where \(y\) is longitudinal path distance or heading, \(u\) is an input voltage, \(b_0\) is a nominal input gain, and \(f\) collects unmodeled dynamics and disturbances. This is a controller model, not a complete nonholonomic or actuator model. The two channels do not model all wheel, battery, chassis, and lateral dynamics as a coupled plant.

The configured initial values are `drive_b0 = 16.0 (in/s^2)/V`, `turn_b0 = 200.0 (deg/s^2)/V`, drive settling parameter `0.6 s`, and turn settling parameter `0.5 s` in [src/tuning.cpp](src/tuning.cpp). The code converts these to internal mm/rad units. They are software defaults, not experimental identification results. The measured gains, units, identification method, uncertainty, and final parameter values used in trials are `TODO`.

## III. Controller and Observer

Let \(r_1,r_2,r_3\) denote reference position, velocity, and acceleration; let \(z_1,z_2,z_3\) denote estimated position, velocity, and lumped disturbance. The implementation uses the estimation error \(e=z_1-y\) and forward-Euler updates:
\[
\begin{aligned}
 z_1[k+1]&=z_1[k]+T_s(z_2[k]-3\omega_o e[k]),\\
 z_2[k+1]&=z_2[k]+T_s(z_3[k]+b_0u_a[k]-3\omega_o^2e[k]),\\
 z_3[k+1]&=z_3[k]+T_s(-\omega_o^3e[k]),
\end{aligned}
\]
where \(u_a\) is the applied-input feedback and \(T_s\) is the measured loop interval. The source clamps \(z_3\) to a configured symmetric bound when that bound is positive. This is a numerical clamp; it is not a proof that the observer is stable, eliminates peaking, or prevents every form of windup.

The feedback law is
\[
 u_c={\omega_c^2(r_1-z_1)+2\omega_c(r_2-z_2)+r_3-z_3\over b_0},
 \qquad \omega_c={4\over T_{settle}},\quad \omega_o=\rho\omega_c,
\]
with configured observer ratio \(\rho=4\) by default and voltage limited by the configured maximum. The forward-Euler observer, sample-time variation, voltage saturation, disturbance clamp, and sensor noise all affect behavior; the continuous-time bandwidth parameterization must not be presented as a stability proof for the implemented closed loop.

For path following, the reference heading is the Bézier tangent angle minus a signed proportional cross-track correction, faded toward zero at low reference speed and clamped to a maximum correction. The reference angular rate is computed as path speed times path curvature. Longitudinal and angular LADRC outputs are mixed as \(u_L=u_{long}+u_{turn}\), \(u_R=u_{long}-u_{turn}\). If either side exceeds the available voltage, both are proportionally scaled. The drive controller feeds the resulting common and differential components back to the respective observers. In-place turns instead apply equal and opposite voltages to the two sides and feed the applied turn voltage to the heading observer.

## IV. Reference Generation and Path Geometry

For signed displacement \(L\), duration \(T\), and normalized time \(\tau=t/T\), the scalar time-scaling function is
\[
 s(t)=L(10\tau^3-15\tau^4+6\tau^5),\quad 0\leq t\leq T.
\]
Its velocity and acceleration are
\[
 \dot s={L\over T}(30\tau^2-60\tau^3+30\tau^4),\qquad
 \ddot s={L\over T^2}(60\tau-180\tau^2+120\tau^3).
\]
The implementation chooses \(T\) as the maximum of the velocity-bound duration \(1.875|L|/v_{max}\), acceleration-bound duration \(\sqrt{(10\sqrt3/3)|L|/a_{max}}\), and a 50-ms floor. The profile has zero endpoint velocity and acceleration. Its peak acceleration magnitude is \((10\sqrt3/3)|L|/T^2\); jerk is finite, but the configured jerk values are not enforced by `QuinticProfile::Configure` and must not be described as an active jerk constraint.

A cubic Bézier curve is represented by four control points. The implementation samples parameter space into a fixed lookup table, accumulates chord lengths for approximate arc-length queries, and estimates maximum curvature over those samples. It iteratively lengthens handles to attempt to satisfy the configured minimum radius, then limits path speed using \(v\leq\sqrt{a_{lat,max}/|\kappa|_{max}}\). These are discretized software calculations, not formal continuous-curve guarantees. Cross-track correction is proportional and heading-based; it is not a demonstrated convergence result.

## V. Experimental Method

### A. Research question and comparator

`TODO: State one primary question and preregistered primary outcome.` The recommended initial study is repeatable straight and curved path tracking with LADRC compared against a clearly specified, properly tuned drivetrain baseline. The lift PID elsewhere in the repository is not a drivetrain comparator. Keep hardware, sensors, reference trajectories, test conditions, tuning effort, and stopping criteria comparable. State whether tuning is performed on separate data from evaluation trials.

### B. Apparatus and calibration

Record robot configuration, wheel and tracking-wheel dimensions, track width, payload and center of mass, battery voltage at each run, surface, sensor model and calibration, software commit, SDK/firmware, controller parameters, and actual loop timing (`TODO`). Identify \(b_0\) for each axis with a documented procedure and report uncertainty. Use an independent ground-truth measurement system with stated accuracy; odometry used by the controller is not independent ground truth.

### C. Protocol and analysis

Use a safe, marked test area and a repeatable start pose. Randomize or counterbalance controller and condition order where practical. Predefine exclusions and report every attempted run. Collect enough independent repetitions for the planned uncertainty analysis (`TODO: sample-size rationale`). Evaluate path error against independent ground truth, endpoint error, settling time under a stated tolerance-and-hold rule, saturation fraction, and voltage/battery conditions. Report units, sample counts, distributions, uncertainty intervals, and the statistical method. Treat multiple samples within a run as correlated; do not treat each control tick as an independent trial.

The existing SD-card logger records autonomous controller/odometry samples for drive, Bézier-drive, and turns. Its CSV includes segment, motion, declared units, references, estimates, voltage, and path/pose fields where applicable. It does not contain a unique trial ID, a full controller-parameter snapshot, independent ground truth, or a rich condition record; the experiment and condition labels in [src/autons.cpp](src/autons.cpp) are simple constants. The analyzer in [scripts/analyze_telemetry.py](scripts/analyze_telemetry.py) calculates descriptive reference-versus-estimate RMSE, maximum absolute error, duration, and maximum estimated disturbance, grouped by motion, unit, and segment. These metrics are not physical accuracy measurements. Turn and drive units must remain separate. Validate CSV output on the Brain and inspect files for truncation before data collection.

### D. Results

**No physical experiment results are available in this draft.** Complete only from validated measurements:

- `TODO: Number of completed runs per controller and condition; exclusions and reasons.`
- `TODO: Table of endpoint and path-tracking outcomes with units, uncertainty, and independent ground-truth method.`
- `TODO: Saturation, battery, actual loop timing, and observer behavior summaries.`
- `TODO: Baseline comparison and preregistered statistical analysis.`

Do not populate these fields with simulated, host-test, illustrative, or generated values. If the proposed comparison is not completed, remove the comparative-results framing and present the paper explicitly as a software/implementation note, subject to the chosen venue's scope.

## VI. Limitations and Discussion

The implementation is a useful experimental platform, but evidence is needed before claiming robustness or superiority. The per-axis plant abstraction omits explicit wheel-level dynamics, and odometry may share sensor errors with the controller feedback. Euler discretization and bandwidth choice require evaluation at measured sample intervals. Saturation handling proportionally scales the mixed wheel commands; it does not ensure that each axis can independently realize its requested input. Disturbance clamping bounds an internal estimate but cannot establish closed-loop stability. The fixed sampled Bézier table approximates arc length and curvature. The current telemetry does not record independent ground truth or a complete trial manifest. The joystick shaping code is not evidence of improved teleoperation usability, and no human-subject study is reported.

A complete paper must include a meaningful novelty statement relative to prior ADRC and robot path-tracking work, validated calibration, a fair comparator, repeated hardware trials, uncertainty analysis, and venue-compliant manuscript formatting. The claimed contributions should be narrowed to what those results establish.

## VII. Conclusion

This draft describes a bandwidth-parameterized LADRC implementation with a third-order Euler ESO, bounded disturbance estimate, quintic scalar time scaling, sampled Bézier path geometry, cross-track heading correction, and applied-voltage feedback. The source supports these implementation descriptions, not claims of experimental superiority or guaranteed stability. Physical validation and comparative evidence remain outstanding. `TODO: Replace this conclusion after experiments; summarize only findings supported by the reported data.`

## References

[1] J. Han, “From PID to active disturbance rejection control,” *IEEE Transactions on Industrial Electronics*, vol. 56, no. 3, pp. 900–906, Mar. 2009, doi: [10.1109/TIE.2008.2011621](https://doi.org/10.1109/TIE.2008.2011621).

[2] Z. Gao, “Scaling and bandwidth-parameterization based controller tuning,” in *Proc. American Control Conference*, vol. 6, 2003, pp. 4989–4996, doi: [10.1109/ACC.2003.1242516](https://doi.org/10.1109/ACC.2003.1242516).

[3] T. Flash and N. Hogan, “The coordination of arm movements: An experimentally confirmed mathematical model,” *The Journal of Neuroscience*, vol. 5, no. 7, pp. 1688–1703, 1985, doi: [10.1523/JNEUROSCI.05-07-01688.1985](https://doi.org/10.1523/JNEUROSCI.05-07-01688.1985).

> Verify each reference against its publisher record and the selected venue's required style before submission. The equations and implementation-specific statements above were checked against the repository source; this is not a peer-reviewed stability analysis or a report of completed experiments.
