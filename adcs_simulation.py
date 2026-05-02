# Run the CubeSat attitude control demonstration.
# This script executes the closed-loop quaternion attitude regulation
# scenario and saves the resulting plots to the plots/ directory by default.

from __future__ import annotations
import argparse
from pathlib import Path
from aocs.scenarios import run_control_demo


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Run the CubeSat attitude control simulation."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("plots/adcs_simulation.png"),
        help="Path where the generated figure will be saved.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the figure window in addition to saving the plot.",
    )
    return parser.parse_args()


def main() -> None:
    """Run the control simulation and print a short summary."""
    args = parse_args()

    results = run_control_demo(
        save_path=args.output,
        show=args.show,
    )

    print("Attitude control simulation completed.")
    print(f"Plot saved to: {results['plot_path']}")
    print(f"Final attitude error: {results['final_attitude_error_deg']:.3f} deg")
    print(f"Final angular-rate norm: {results['final_rate_norm']:.6f} rad/s")


if __name__ == "__main__":
    main()
