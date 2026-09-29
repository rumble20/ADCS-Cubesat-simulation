# Assumptions and simplifications

I kept everything as simple as I could so I can explain every part of it.

## Spacecraft and dynamics
- Rigid body, constant diagonal inertia (numbers guessed for a 3U CubeSat).
- Attitude with quaternions, scalar first.
- Simple Euler integration with a fixed step of 0.1 s.

## Orbit and environment
- Circular orbit at 500 km, 97.4 deg inclination. No drag, no J2.
- Sun direction fixed for the whole run (it moves less than 0.1 deg per orbit).
- Eclipse: simple cylinder shadow behind the Earth, no half-shadow.
- Magnetic field: tilted dipole. Only its direction is used.
- Disturbance torques: made-up sine waves around 1e-5 N m.

## Actuators
- Three ideal reaction wheels on the body axes.
- Torque limit and momentum limit. No friction, no motor model.
- No magnetorquers, so no momentum dumping.

## Sensors
- Sun sensor and magnetometer: true direction + Gaussian noise, normalised.
  The sun sensor can see the sun from any direction (no field of view).
- Gyro: true rate + bias + white noise, the bias slowly random-walks.
- All sensors at every step (10 Hz), no delays.
- The satellite knows the true sun and field directions in the inertial
  frame. A real one would use a model with errors in it.

## Estimation and control
- TRIAD only to start the filter.
- MEKF with 6 states: small attitude error (3) and gyro bias (3).
- PD controller on the estimated attitude, with a cap on the slew rate.
- The target attitude is fixed in the inertial frame (no nadir pointing).
