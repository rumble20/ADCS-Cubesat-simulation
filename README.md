# CubeSat ADCS simulation (side project)

A small attitude determination and control simulation for a CubeSat that I
build up step by step to learn the basics of ADCS. It is a personal learning
project, not flight software, and it is written in plain MATLAB functions so
every part is easy to follow. `NOTES.md` is the log of what went wrong and
what I changed along the way.

## What it simulates
- A 3U-size CubeSat in a 500 km circular orbit, going through eclipse
- Three reaction wheels with torque and momentum limits
- Sun sensor, magnetometer and gyro (with noise and a drifting bias)
- TRIAD to get a first attitude, then a MEKF that estimates attitude and gyro bias
- A PD controller that points the satellite using only the estimated attitude
- A requirements check (PASS/FAIL) and a 50-run Monte Carlo

## Repository
```text
matlab/              the simulation (start from run_adcs.m)
tests/               quick checks for the quaternion functions and TRIAD
docs/requirements.md what the simulation should achieve
docs/assumptions.md  what is simplified
docs/code_guide.md   how the code works and in which order to read it
python_reference/    Python copy of the early wheel + sensor version (used to cross-check)
adcs_simulation.py   first Python version (control only)
ekf_estimation.py    first Python version (1-axis EKF)
NOTES.md             bugs, surprises, decisions
```

## How to run
In MATLAB (also works in GNU Octave):
```matlab
cd matlab
run_adcs        % one orbit (~95 min simulated), prints the requirements check
monte_carlo     % 50 short runs with random start conditions, takes a few minutes
```
Plots are saved in `plots/`.

Tests:
```matlab
cd tests
test_quaternions
```

## Status
- [x] Requirements and PASS/FAIL check
- [x] Reaction wheels
- [x] Sensors
- [x] TRIAD + MEKF with gyro bias
- [x] Monte Carlo
- [x] Circular orbit, field direction, eclipse
- [ ] Simulink version
- [ ] Monte Carlo with the orbit (only run before the orbit was added)
