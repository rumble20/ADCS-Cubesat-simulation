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

## Useful reading
The MEKF and TRIAD follow the textbook by Markley and Crassidis,
*Fundamentals of Spacecraft Attitude Determination and Control* (2014),
mostly the chapters on attitude estimation. Wertz, *Spacecraft Attitude
Determination and Control*, is the classic for the environment models.
