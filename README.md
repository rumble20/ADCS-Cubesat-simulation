# CubeSat AOCS Demonstrator

This repository contains a compact Attitude and Orbit Control System (AOCS) demonstrator for a CubeSat-class spacecraft. In practice, the current scope is attitude-focused: rigid-body rotational dynamics, closed-loop control, sensor simulation, and state estimation.

The project is meant as a portfolio piece and learning exercise. It is not a flight-ready AOCS stack. The goal is to show sound fundamentals, working simulations, readable code, and a realistic path toward more advanced spacecraft GNC work.

## Project scope

The repository currently includes:

- 3-axis rigid-body attitude dynamics
- quaternion-based attitude propagation
- quaternion PD attitude control
- simple disturbance torque injection
- simulated sensors:
  - gyroscope with bias and noise
  - sun direction measurement
  - magnetic field direction measurement
- TRIAD-based coarse attitude determination
- EKF-based attitude and gyro bias estimation
- plot generation for both control and estimation scenarios

## Why this project exists

This project was built to demonstrate practical interest in spacecraft AOCS through code rather than only theory. It is deliberately scoped as a serious side project: strong on fundamentals, honest about simplifications, and structured so that each part is easy to explain in an interview.

It is intended to show interest and ability in:

- spacecraft attitude dynamics
- closed-loop control
- sensor modelling
- state estimation
- engineering-oriented Python development
- technical documentation

## Repository structure

```text
cubesat-aocs/
├── aocs/
│   ├── __init__.py
│   ├── control.py
│   ├── dynamics.py
│   ├── ekf.py
│   ├── math_utils.py
│   ├── scenarios.py
│   └── sensors.py
├── docs/
│   ├── assumptions.md
│   └── design_notes.md
├── plots/
├── tests/
│   └── smoke_test.py
├── adcs_simulation.py
├── ekf_estimation.py
├── requirements.txt
├── .gitignore
└── README.md

## Quick start
Install dependencies numpy and matplotlib

Run the control simulation:

'''bash
python adcs_simulation.py
'''

Run the estimation simulation:

'''bash
python ekf_estimation.py
'''

Both scripts save figures into the plots/ directory.

## Simulation overview

### 1. Attitude dynamics and control
adcs_simulation.py simulates a CubeSat with a fixed inertia matrix, an initial attitude error, and nonzero angular velocity. A quaternion PD controller drives the spacecraft toward a reference attitude while small disturbance torques act on the body.

Outputs include:
- attitude error over time
- body angular rates
- commanded control torques

### 2. Attitude estimation
ekf_estimation.py simulates a tumbling spacecraft observed through:
- a biased gyroscope
- a noisy sun-direction measurement
- a noisy magnetic-field measurement

A TRIAD solution provides a coarse attitude estimate from vector observations, and an EKF refines the estimate while tracking gyro bias.

Outputs include:
- attitude estimation error
- true angular rates and gyro measurements
- true and estimated gyro bias

## Modelling assumptions

This is an educational demonstrator. The main simplifications are:
- only attitude dynamics are modelled
- orbit propagation is not included
- inertial reference vectors are fixed instead of orbit-dependent
- the controller commands body torque directly
- actuator dynamics are not modelled
- disturbances are simple analytical torques
- the estimator uses TRIAD as a measurement source rather than a complete sensor-processing chain

These simplifications are intentional. They keep the project compact and readable while still covering the main estimation-and-control loop.

Even though the project is intentionally modest, it contains several concepts that are directly relevant to entry-level AOCS and GNC work, like rigid-body rotational dynamics, quaternion kinematics, quaternion feedback control, noisy sensor simulation, EKF covariance propagation and correction exc.

## Limitations

There a lot of other factores which haven't been considered in the project and limit how realistically this simulates the proposed environment. The project doesn't (yet) include:
- orbit propagation
- LVLH or mission-dependent reference genera,tion
- reaction wheels or magnetorquers
- actuator saturation and momentum storage
- eclipse handling
- realistic magnetic field or sun ephemeris models
- high-fidelity disturbance torques
- Monte Carlo analysis
- real-time or embedded software constraints

## Possible next steps

1) replace direct torque control with a reaction wheel or magnetorquer model
2) add an orbit propagator and time-varying reference/environment vectors
3) include actuator saturation and sensor update rates
4) compare TRIAD + EKF against QUEST or a full MEKF
5) add Monte Carlo campaigns and summary performance metrics

## Notes for reviewers
The aim of this project is to demonstrate a genuine interest in AOCS with code that is easy to inspect, discuss, and extend. It is intentionally more structured than a classroom script and intentionally less ambitious than a flight-software claim.
