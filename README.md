# CubeSat AOCS Mini Project

This repository contains a small side project built to explore some basic Attitude and Orbit Control System concepts for a CubeSat-type spacecraft.

The project is intentionally compact. Instead of trying to build a full software framework, I kept it as two standalone Python scripts:

- one for attitude dynamics and control
- one for attitude estimation with an Extended Kalman Filter

The goal is not to present flight-ready software, but to show practical interest in AOCS and a working understanding of some of the main ideas: rigid-body dynamics, quaternion-based control, noisy measurements, and state estimation.

## Project contents

```text
.
├── adcs_simulation.py
├── ekf_estimation.py
├── README.md
├── requirements.txt
├── plots/
└── docs/
```

## What each script does
### adcs_simulation.py

This script simulates a simple 3-axis CubeSat attitude control problem.

It includes:
- rigid-body rotational dynamics
- quaternion attitude propagation
- quaternion PD control
- small disturbance torques
- plots of:
  - attitude error
  - angular velocity
  - control torque
  - quaternion components

The purpose of this part is to show a simple closed-loop attitude regulation problem.

### ekf_estimation.py

This script simulates a simple 1-axis attitude estimation problem.

It includes:
- a 1-axis rotational model
- simulated gyro and sun-sensor-like measurements
- an EKF for estimating:
  - angle
  - angular velocity
  - gyro bias
- plots of:
  - true vs estimated angle
  - true vs estimated angular velocity
  - true vs estimated gyro bias
  - angle estimation error with covariance bounds

The purpose of this part is to show a simple sensor-fusion and estimation problem without making the code too heavy.

## Why the project is split this way

The control simulation is done in 3 axes because attitude control is much more meaningful there, especially with quaternions.
The estimator is kept to 1 axis on purpose. A full 3-axis estimator would be more realistic, but it would also make the project much larger and less transparent. For this repo, I preferred a smaller implementation that is easy to read and explain.

## How to run

Run the control simulation:
```bash
python adcs_simulation.py
```

Run the estimation simulation:
```bash
python ekf_estimation.py
```

Both scripts save figures in the plots/ directory.

## What this project is meant to show

This repository is mainly a learning and portfolio project. It is meant to show that I took the initiative to experiment with topics that are relevant to entry-level AOCS and GNC work, including:
- rotational dynamics
- quaternion kinematics
- basic feedback control
- disturbance rejection
- sensor modelling
- Extended Kalman Filtering
- gyro bias estimation

## Main simplifications

This is a deliberately simplified project. In particular:

- orbit propagation is not modelled
- the control case uses direct torque rather than a detailed actuator model
- the disturbance torques are simple analytical functions
- the EKF example is 1-axis rather than full 3-axis
- environmental models are very basic

These simplifications are intentional. The aim was to build something small, readable, and functional rather than a large unfinished simulator.

## Possible next steps

Some natural extensions would be:
- reaction wheel or magnetorquer modelling
- a 3-axis attitude estimator
- more realistic disturbance models
- actuator saturation and sensor update-rate effects
- Monte Carlo runs for sensitivity analysis
- comparison between different control laws