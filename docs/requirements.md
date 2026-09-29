# Requirements

These are my own requirements for the simulation, chosen to be realistic for a small
CubeSat doing simple nadir-style pointing. I picked the numbers by reading about typical
CubeSat pointing accuracy, so they are reasonable guesses, not a real mission spec.
`matlab/check_requirements.m` checks them at the end of every run.

| ID | Requirement | Limit | Notes |
|----|-------------|-------|-------|
| REQ-01 | Pointing error after t = 60 s | < 2 deg | worst value for all t >= 60 s |
| REQ-02 | Body angular rate after t = 60 s | < 0.01 rad/s | norm of the rate vector |
| REQ-03 | Peak reaction wheel momentum | < 80 % of capacity | keeps margin before saturation |
| REQ-04 | Sun sensor error (RMS) | < 0.5 deg | angle between measured and true sun vector |
| REQ-05 | Magnetometer error (RMS) | < 1.0 deg | angle between measured and true field vector |

## Conditions
- Initial error is about 60 deg with a small initial rate (see `adcs_params.m`).
- Sinusoidal disturbance torques of about 1e-5 to 2e-5 N m.
- Wheels: 1 mN m torque and 5 mN m s momentum per axis.

## Not covered yet
- Estimation accuracy (needs the attitude estimator).
- Power, thermal, orbit, eclipse.
- Momentum dumping.
