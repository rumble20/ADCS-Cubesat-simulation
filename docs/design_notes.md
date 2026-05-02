# Design notes

## What was improved from the original scripts

The starting point already had the right themes: quaternion dynamics, PD control, noisy measurements, and filtering. The main issues were consistency, scope, and structure.

This version improves the project in the following ways:

1. The repository is modular.
   - math, dynamics, control, sensors, and estimation are split into separate modules
   - top-level scripts are reduced to small entry points

2. The control logic is corrected.
   - quaternion error is computed with quaternion algebra instead of element-wise multiplication
   - gains are defined outside the simulation loop

3. The estimation case is made more representative of the AOCS domain.
   - the original 1D EKF is replaced by a 3-axis attitude and gyro-bias estimator
   - attitude measurements come from sun and magnetic-field vector observations through TRIAD

4. The simulations produce repository-ready outputs.
   - plots are saved automatically
   - scripts print short terminal summaries

5. The documentation is aligned with the implementation.
   - scope, assumptions, and limitations are stated explicitly
   - the README matches the actual repository contents

But this repository is still presented as a side project, not a claim of production-grade flight software. The code stays compact on purpose.