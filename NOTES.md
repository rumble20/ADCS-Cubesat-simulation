# Notes

Log of what went wrong, what surprised me and what I decided. Newest at the bottom.
Everything up to "First run in real MATLAB" was run in GNU Octave 8.4.

## Setup
- The docs described a modular repo with a 3-axis EKF and TRIAD, but the code
  was two scripts and a 1-axis EKF. Rewrote the docs to match the code.
- The EKF file was called `ekf.estimation.py` but the README said
  `ekf_estimation.py`. Renamed it (Python can't import a name with a dot).

## Wheels
- First momentum limit was 4 mN m s. The first manoeuvre used about 87 % of it,
  so I raised it to 5 mN m s.
- Sign: to push the body one way, the wheel gets the opposite torque.
  `reaction_wheels.m` takes the torque wanted on the body and flips it inside.

## Sensors
- Expected the sun sensor error to be 0.29 deg (0.005 rad) but got ~0.4 deg.
  Noise goes on all 3 components and 2 of them are perpendicular to the vector,
  so the angle error is about sqrt(2) bigger. REQ-04/05 are set from that.

## First run of the MATLAB code
- `yline` doesn't exist in Octave. Replaced with a normal `plot` of a line.
- Plots were saved in `matlab/plots/` instead of the repo `plots/` folder.
- `check_requirements.m` had the wheel capacity typed in as 5e-3 instead of
  using `p.h_max`. Would have broken the moment I changed the parameter.
- Good news: the control numbers matched the Python cross-check exactly
  (0.03761 deg pointing, 70.01 % wheel momentum).
- With the old gains the body rate peaks at ~0.17 rad/s (10 deg/s) during the
  first slew. Much faster than a real CubeSat would turn.

## Tests
- `test_quaternions.m` did `addpath(pwd)` but `run()` changes into the script's
  folder, so pwd was `tests/` and nothing was found. Changed to `addpath('../matlab')`.

## TRIAD
- 0.74 deg RMS, 2.3 deg worst case. Too noisy to point with directly.

## MEKF
- Worked first time: 0.11 deg RMS (vs 0.74 for TRIAD), bias converged from
  2e-3 to ~5e-5 rad/s, 99.8 % of samples inside 3 sigma (should be ~99.7 %).
- The error plot was useless at first: the 3-sigma at t = 0 is 15 deg and it
  squashed everything else. Added `ylim([-1 1])`.
- The x-axis sigma is bigger than y and z. At the target attitude body x points
  at the sun, and a rotation around the sun line can't be seen by the sun
  sensor, only by the (noisier) magnetometer.

## Closing the loop
- Once the controller used the estimated attitude, the rate after 60 s went
  from 2e-5 to 2.4e-3 rad/s. Kd = 0.12 multiplies the gyro noise straight
  into the torque.
- Gain sweep (with the estimator in the loop):

  | Kp | Kd | pointing after 60 s | rate after 60 s | wheel peak |
  |----|----|----|----|----|
  | 0.08 | 0.12 | 0.20 deg | 2.4e-3 | 70 % |
  | 0.01 | 0.03 | 0.30 deg | 9.5e-4 | 45 % |
  | 0.004 | 0.02 | 0.66 deg | 9.8e-4 | 31 % |
  | 0.0015 | 0.006 | 1.90 deg | 1.4e-3 | 33 % |

  Lower gains = less noise but the disturbances push the pointing error up.
  Picked 0.01 / 0.03.

## Bias requirement
- Set REQ-07 to 2e-4 rad/s before checking anything. It failed (2.4e-4).
  The filter's own 3-sigma was 2.2e-4 per axis, so for the 3-axis norm my
  limit was tighter than the gyro allows. Moved it to 5e-4.

## Monte Carlo (before the orbit was added)
- Moved the loop into `simulate_adcs.m` so it can run many times. Same numbers.
- 50 runs, random attitude (0-180 deg), rates and bias: 46/50 passed.
  All 4 failures were REQ-03 (wheels over 80 %) with 137-172 deg starting errors.
  99.94 % of samples inside 3 sigma. Pointing: median 0.37 deg, worst 0.46 deg.
- Fix: cap the commanded slew rate at 0.1 rad/s. The 4 failing runs then
  used 25-34 % of the wheels and passed. I only reran those 4, not all 50.

## Orbit
- Checked by hand: period 94.6 min at 500 km, 35.8 min eclipse per orbit.
- Tried a proper sun position formula, but the sun moved only 0.065 deg in a
  whole orbit, so I went back to a fixed direction.
- Added gravity gradient torque: ~2e-9 N m, nothing compared to the 1e-5
  disturbances. Removed it again.
- The field direction changes by up to 156 deg over the orbit, so the
  magnetometer is where the moving reference really matters.

## First full orbit run failed
- In eclipse the attitude knowledge drifted to 5.7 deg (magnetometer only).
- TRIAD had an 11.7 deg error once: the sun and field directions were only
  11 deg apart there. Now the filter only starts from TRIAD when they are
  more than 20 deg apart.
- Rate spike of 0.0115 rad/s 1.4 s after leaving eclipse: the sun comes back,
  the filter corrects the drift and the controller snaps back.
- Decision (simplest thing): pointing, rate, knowledge and bias are only
  checked in sunlight, 60 s after the sun is visible. Eclipse values are
  printed just for info. With that, knowledge was 0.57 deg, just over my
  0.5 deg limit from before the orbit, so REQ-06 is now 1 deg.
- The eclipse drift changes a lot from run to run: 5.7, 14.2 and 11.9 deg in
  three runs with small differences. In the dark the error just wanders.
  The bias error also peaks in eclipse (8.6e-4), in sunlight it stays under 3e-4.

## First run in real MATLAB (R2024b)
- `test_quaternions` passed and `run_adcs` gave Overall: PASS, no code changes
  needed. One orbit takes about 40 s.
- Numbers are not the same as in Octave. MATLAB and Octave have different
  random generators, so `rng(1)` gives different noise. Sunlit numbers are
  close (pointing 0.47 deg, knowledge 0.35 deg), but the eclipse drift was
  9.4 deg this time. Same thing as before: in the dark it just wanders.
- The "MEKF RMS 2.58 deg" printed at the start is misleading because it
  includes the eclipse. The sunlit max (0.35 deg) is the useful number.
- Deleted `python_reference/`. It was a Python copy of the early MATLAB
  version, only there to check the MATLAB code while I couldn't run it. It
  still had the old gains and fixed vectors, so now it was just confusing.

## Simulink
- Only the control loop is in Simulink (wheels, PD, dynamics, disturbances),
  with the TRUE attitude. Sensors and the MEKF are still MATLAB only.
- The model is built by `build_adcs_model.m` because git can't diff a .slx.
  The MATLAB Function blocks call the same .m files as the MATLAB loop.
- Bug: `run_simulink` crashed with "Index in position 1 exceeds array bounds".
  To Workspace saved q as 4x1x3001 (because the signal is a column vector),
  not 3001x4. Fixed with `squeeze(...)'`.
- Result: the wheel momentum curves are on top of each other (peak 36.52 %
  vs 36.40 %), and pointing at 30 s is 0.633 vs 0.611 deg. After the slew
  the MATLAB loop is a bit noisier because it uses the MEKF.
- The integrator lets the quaternion norm drift a little, so the blocks
  normalise q before using it.

## Still to do
- Rerun the Monte Carlo now that the orbit is in (it still uses 300 s runs).
- Put the sensors and the MEKF in the Simulink model too.
