# Code guide

How the MATLAB code is organised and what each part does, so someone new
(or me in six months) can follow it.

## The loop
Every 0.1 s, `simulate_adcs.m` does:

```
orbit position -> field direction, eclipse?
      |
sensors: sun (if not eclipse), magnetometer, gyro
      |
MEKF: predict with gyro, update with sun and magnetometer
      |
PD controller on the ESTIMATED attitude -> wanted torque
      |
reaction wheels: apply limits -> real torque on the body
      |
dynamics: update rate and quaternion
```

The controller never sees the true attitude, only the estimate. The true
values are used only for logging and for the requirements check.

## Suggested reading order

1. `adcs_params.m` - all the numbers. Quick look.
2. `quat_mult.m`, `quat_conj.m`, `quat_error.m`, `small_rotation_quat.m`,
   `quat_to_dcm.m` - quaternion helpers. `tests/test_quaternions.m` checks them.
3. `simulate_adcs.m` - the main loop. Read the dynamics part at the bottom
   first (Euler's equation with the wheel momentum), then go up.
4. `reaction_wheels.m` - the torque sign flip and the two limits.
5. `sun_sensor.m`, `magnetometer.m`, `gyro_model.m` - noise models.
6. `triad.m` + `dcm_to_quat.m` - attitude from two vectors.
7. `mekf_predict.m`, `mekf_update.m` - the filter (see below).
8. `orbit_position.m`, `mag_field_eci.m`, `in_eclipse.m` - environment.
9. `check_requirements.m`, `run_adcs.m`, `monte_carlo.m` - checks and plots.

## The ideas, briefly

**Quaternions.** 4 numbers for a rotation without the problems of Euler
angles. `q_new = q * dq` rotates by a small extra rotation measured in the
body frame (that is how the gyro sees it).

**PD control.** Torque = -Kp * (vector part of the error quaternion)
- Kd * (rate). Written here as "the error gives a wanted rate, capped at
`w_max`, and Kd pushes the real rate towards it". For small errors it is
the same PD. The cap stops big slews from filling up the wheels.

**Reaction wheels.** To turn the body one way you spin a wheel the other
way. The total momentum stays the same, so the wheel momentum `h` also
appears in the dynamics (`cross(w, I*w + h)`).

**TRIAD.** With two known directions (sun and field) measured in the body
and known in space, you build a frame from each pair and the rotation
between them is the attitude. Trusts the first vector more. Bad when the
two vectors are nearly parallel.

**MEKF (multiplicative EKF).** Instead of filtering the 4 quaternion numbers
(which must keep norm 1), the filter keeps the quaternion outside and only
estimates a small 3-number rotation error plus the gyro bias (6 states).
- Predict: rotate the quaternion with (gyro - bias estimate), grow P.
- Update: compare the measured vector with the one predicted from the
  current attitude. The difference goes through the Kalman gain into a
  small rotation (applied to the quaternion) and a bias correction.
- `H = [skew(b_pred), 0]` comes from: a small rotation dtheta changes a
  vector b by about `skew(b) * dtheta`.

**Eclipse.** No sun means only the magnetometer, and rotation around the
field line cannot be seen. The filter knows this: its sigma grows in eclipse.

## The Simulink model
`simulink/adcs_model.slx` is only the control loop: the controller, the two
wheel limits, the rigid body dynamics and the disturbances. It uses the TRUE
attitude and rate, there are no sensors or MEKF in it yet.
- `build_adcs_model.m` creates the model block by block. Read this instead
  of clicking through the .slx, it is the same thing in text.
- The MATLAB Function blocks call the same files as the MATLAB loop
  (`quat_error`, `quat_mult`, `disturbance_torque`), and all the numbers
  come from `p` (from `adcs_params`) in the workspace.
- The main difference from `simulate_adcs.m`: Simulink integrates with
  Integrator blocks and the ode4 solver, the MATLAB loop does it by hand.
- `run_simulink.m` runs it for 300 s and plots it on top of the MATLAB loop
  (`plots/simulink_vs_matlab.png`). The wheel momentum should be almost the
  same in both.

## Glossary
- **ECI**: inertial frame centred on the Earth, does not rotate. The sun and
  field directions are known in this frame.
- **Body frame**: axes fixed to the satellite. Sensors measure in this frame.
- **Attitude**: the rotation between ECI and body. Stored as a quaternion `q`.
- **DCM / `A`**: the same rotation as a 3x3 matrix. `A * r_eci = r_body`.
- **Quaternion**: 4 numbers `[q0; q1; q2; q3]`, scalar first. `q0 = cos(angle/2)`,
  so the pointing error in degrees is `2*acosd(q0)` of the error quaternion.
- **Gyro bias**: a slowly changing offset in the gyro reading. If not
  estimated, integrating the gyro makes the attitude drift.
- **TRIAD**: attitude from two vector measurements, no memory.
- **MEKF**: Kalman filter that uses the gyro between measurements and
  estimates the bias too. Much smoother than TRIAD.
- **P, sigma, 3-sigma**: the filter's own guess of its error. If the filter
  is right, the real error stays inside +-3 sigma about 99.7 % of the time.
- **Eclipse**: the part of the orbit in the Earth's shadow (~36 min of 95).
- **Wheel momentum `h`**: how fast the wheels spin. It has a maximum, and
  without magnetorquers the only way to reduce it is to turn back.
- **Monte Carlo**: run the same simulation many times with random start
  conditions, to see if it works in general and not just for one case.

## Things to try (to understand it)
Numbers below are from 300 s runs (`p.t_final = 300`), base case first:
pointing 0.39 deg, rate 9.7e-4 rad/s, wheels 36 %, bias error 1.7e-4 rad/s.
- Set `p.P0_bias = 1e-12`: the filter is told it already knows the bias
  (it thinks it is zero), so it learns it very slowly. Bias error goes to
  6.0e-4 (REQ-07 fails) and pointing gets worse (0.95 deg).
- Set `p.Kp = 0.08` and `p.Kd = 0.12` (the first values): pointing gets
  tighter (0.17 deg) but the rate more than doubles (2.3e-3), because the
  gyro noise goes straight into the torque.
- Set `p.h_max = 2e-3`: the first slew uses 91 % of the wheels, REQ-03 fails.
- Comment out the magnetometer update in `simulate_adcs.m`: in eclipse the
  filter then has nothing at all and sigma grows on all axes.
- In `adcs_params.m` change `p.raan` to 96 deg: the orbit no longer goes
  through the shadow and all the requirements are checked all the time.

## Useful reading
The MEKF and TRIAD follow the textbook by Markley and Crassidis,
*Fundamentals of Spacecraft Attitude Determination and Control* (2014),
mostly the chapters on attitude estimation. Wertz, *Spacecraft Attitude
Determination and Control*, is the classic for the environment models.
