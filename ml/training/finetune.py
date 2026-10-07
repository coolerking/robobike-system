"""Entry point reserved for project-specific SmolVLA fine-tuning."""

import argparse


def main():
    parser = argparse.ArgumentParser(description="Robobike SmolVLA fine-tuning scaffold")
    parser.add_argument("--dataset", required=True, help="Curated LeRobot dataset path or repo ID")
    parser.parse_args()
    raise NotImplementedError(
        "Configure LeRobot SmolVLA training, observation/action mapping and checkpoints first."
    )


if __name__ == "__main__":
    main()
