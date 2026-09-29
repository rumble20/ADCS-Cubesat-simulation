# Requirements

My own requirements for the simulation. I picked the numbers from what I read
about typical CubeSats and then adjusted a couple of them after running the
simulation (the reasons are in NOTES.md). They are not from a real mission.

`matlab/check_requirements.m` checks all of them at the end of every run and
prints PASS/FAIL.

| ID | What | Limit | When it is checked |
|----|------|-------|--------------------|
| REQ-01 | Pointing error | < 2 deg | in sunlight, 60 s after the sun is visible |
| REQ-02 | Body angular rate | < 0.01 rad/s | in sunlight, 60 s after the sun is visible |
| REQ-03 | Peak wheel momentum | < 80 % of max | whole run |
| REQ-04 | Sun sensor error (RMS) | < 0.5 deg | whole run (sunlit samples) |
| REQ-05 | Magnetometer error (RMS) | < 1 deg | whole run |
| REQ-06 | Attitude knowledge error (MEKF) | < 1 deg | in sunlight, 60 s after the sun is visible |
| REQ-07 | Gyro bias estimate error | < 5e-4 rad/s | in sunlight, after 120 s |

## Why "in sunlight"
In eclipse the sun sensor sees nothing and only the magnetometer is left.
One vector is not enough to know the full attitude, so the attitude drifts
by a few degrees until the sun comes back. I did not try to fix that, I just
check the requirements in sunlight and print the eclipse numbers for info.

## Not covered
Power, thermal, momentum dumping, anything about a real mission.
