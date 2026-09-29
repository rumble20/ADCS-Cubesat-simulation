# Modelling assumptions

This project sits between a classroom exercise and a more structured engineering demonstrator.
I keep it simple on purpose so I can explain every part of it.

## Dynamics
- Rigid body, constant diagonal inertia matrix.
- No orbit propagation, attitude only.
- Attitude propagated with quaternions (scalar-first).
- Reaction wheel momentum is included in the rotational equation.

## Control
- Quaternion PD controller computes the torque it wants on the spacecraft.
- Three ideal reaction wheels aligned with the body axes.
- Wheel torque and momentum limits are modelled. Wheel friction, motor dynamics and
  wheel misalignment are not.
- No momentum dumping (magnetorquers), so long runs would eventually saturate.
- The controller uses the true state for now (no estimator in the loop yet).

## Sensors
- Sun sensor and magnetometer: true unit vector in the body frame + Gaussian noise, renormalised.
- Gyro: true rate + bias + Gaussian noise, bias follows a small random walk.
- The inertial reference vectors are fixed, not orbit-dependent.
- No sensor update rates, misalignment, field of view or eclipse.

## Estimation (planned, not built yet)
- TRIAD for a coarse attitude from the sun and magnetic vectors.
- Error-state EKF for attitude error and gyro bias.
