# CubeSat AOCS Mini Project

A side project where I try to build up a small attitude determination and control (ADCS)
simulation for a CubeSat, one piece at a time. I am learning as I go, so expect simplifications
and a few rough edges. See `NOTES.md` for what went wrong along the way.

## What is in here

```text
.
├── adcs_simulation.py       first Python version: 3-axis quaternion PD control
├── ekf_estimation.py        first Python version: 1-axis EKF with gyro bias
├── matlab/                  MATLAB version: wheels, sensors, requirements check
├── python_reference/        Python cross-check of the MATLAB model
├── docs/                    requirements, assumptions, design notes
├── NOTES.md                 bugs, surprises, decisions
└── plots/
```

## Current status
- [x] Requirements defined and checked automatically (`docs/requirements.md`)
- [x] Reaction wheels with torque and momentum limits
- [x] Sun sensor, magnetometer, gyro (noise + bias random walk)
- [ ] TRIAD + 3-axis error-state EKF
- [ ] Monte Carlo runs
- [ ] Simple orbit (moving reference vectors, eclipse)
- [ ] Simulink model

## How to run

MATLAB:
```matlab
cd matlab
run_adcs
```
It prints a PASS/FAIL table for each requirement and saves plots in `plots/`.

Python (original scripts and cross-check):
```bash
python adcs_simulation.py
python ekf_estimation.py
python python_reference/adcs_wheels_sensors.py
```

## Main simplifications
- Attitude only, no orbit propagation
- Ideal wheels (no friction, no motor dynamics), no momentum dumping
- Controller uses the true state for now, sensors are only simulated and logged
- Fixed reference vectors for sun and magnetic field
- Disturbance torques are simple sinusoids

## What I am trying to learn
Rigid-body dynamics, quaternions, PD control, reaction wheels, sensor modelling,
requirements-driven testing, and (next) attitude determination with an EKF.
