# Notes

Running log of decisions, surprises and things I got wrong. Newest at the bottom.
Only things that really happened go in here.

## Setup
- The docs described a modular repo with a 3-axis EKF and TRIAD, but the code was two
  scripts and a 1-axis EKF. Docs and code disagreed. I rewrote the docs to say what is
  actually built and put the rest in a "not done yet" list.
- The EKF file was called `ekf.estimation.py` but the README calls it `ekf_estimation.py`.
  Renamed to match the README. (Python can't import a module with a dot in its name.)

## Wheels
- First version of the wheel model had a 4 mN m s momentum limit. The wheels reached
  about 87 % of that on the first manoeuvre, because the 60 deg initial error gets
  absorbed into wheel momentum. Raised the limit to 5 mN m s (peak is now about 70 %).
  Still a guess for a real wheel, but it shows why momentum sizing matters.
- Sign convention: to torque the body one way, the wheel gets the opposite torque.
  Easy to flip by mistake, so `reaction_wheels.m` takes the torque wanted on the body
  and handles the sign inside.
- Wheels have no momentum dumping. Over 300 s it doesn't matter, over hours it would.

## Sensors
- I expected the sun sensor error to be about 0.29 deg (0.005 rad). It came out at about
  0.4 deg RMS. Reason: noise is added to all 3 components, and 2 of them are
  perpendicular to the true vector, so the angle error is roughly sqrt(2) times bigger.
- Requirements REQ-04 and REQ-05 are therefore set from this measured number with a bit
  of margin, not from the raw sigma.

## Requirements
- REQ-01/02 pass with a lot of margin (error is about 0.04 deg after 60 s). The limits
  are loose because the controller uses the true state. They should get harder to
  meet once the estimator is in the loop, and I will revisit them then.

## MATLAB port
- The MATLAB code was written and cross-checked against the Python version, but the
  Python one was the only one run when this was written. First thing to do: run
  `run_adcs.m` in MATLAB and note here what breaks.
- Simulink model not built yet.

<!-- add your own bugs below as you run the MATLAB code -->
