# Modelling assumptions

This project is intentionally positioned between a classroom exercise and a more structured engineering demonstrator.

## Dynamics

- The spacecraft is treated as a rigid body.
- The inertia matrix is constant and diagonal.
- Translational motion and orbit propagation are not included.
- The attitude state is propagated with quaternions.

## Control

- The controller commands torque directly.
- No actuator dynamics are modelled.
- No wheel momentum storage, motor current limits, or magnetorquer dipole limits are included.

## Sensors

- Gyro noise is Gaussian.
- Gyro bias follows a small random walk.
- Sun and magnetic-field measurements are represented as noisy unit vectors.
- The inertial reference vectors are fixed rather than orbit-dependent.

## Estimation

- TRIAD provides a coarse attitude measurement.
- The EKF estimates small attitude error and gyro bias.
- The estimator is intended to be readable and compact rather than highly optimized.


All these assumptions were kept because the point of the project is to show a complete engineering chain that remains easy to inspect and discuss. 
