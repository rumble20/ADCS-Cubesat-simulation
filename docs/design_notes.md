# Design notes

## Where the project started
Two standalone Python scripts: a 3-axis quaternion PD controller and a 1-axis EKF.
Good start, but the control was direct torque and the EKF was 1-axis only.

## Current state
Done:
- Requirements written down and checked automatically (`docs/requirements.md`)
- Reaction wheel model with torque and momentum limits
- Sun sensor, magnetometer and gyro models
- MATLAB version of the control loop (`matlab/`), with a Python cross-check (`python_reference/`)

Not done yet (roughly in this order):
- TRIAD and a 3-axis error-state EKF
- Monte Carlo runs
- Simple circular orbit so the reference vectors move
- Simulink model of the loop

## Style
Plain functions, one small file each, all parameters in `adcs_params.m`.
No classes. Compact rather than clever. Side project, not flight software.
