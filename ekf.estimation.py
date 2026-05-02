# Run the CubeSat attitude estimation demonstration.
# This script executes the estimator scenario based on:
# - rigid-body attitude propagation
# - gyroscope measurements with bias and noise
# - noisy sun and magnetic-field direction measurements
# - TRIAD coarse attitude determination
# - EKF attitude and gyro-bias estimation

from __future__ import annotations
import argparse
from pathlib import Path
from aocs.scenarios import run_estimation_demo


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Run the CubeSat attitude estimation simulation."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("plots/ekf_estimation.png"),
        help="Path where the generated figure will be saved.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=7,
        help="Random seed used for sensor noise generation.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the figure window in addition to saving the plot.",
    )
    return parser.parse_args()


def main() -> None:
    """Run the estimation demo and print a short summary."""
    args = parse_args()

    results = run_estimation_demo(
        save_path=args.output,
        seed=args.seed,
        show=args.show,
    )

    print("Attitude estimation simulation completed.")
    print(f"Plot saved to: {results['plot_path']}")
    print(f"Final EKF attitude error: {results['final_attitude_error_deg']:.3f} deg")
    print(f"RMS EKF attitude error: {results['rms_attitude_error_deg']:.3f} deg")


if __name__ == "__main__":
    main()
