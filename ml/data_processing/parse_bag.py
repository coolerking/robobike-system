"""Inspect ROS 2 bag metadata without depending on ROS."""

import argparse
from pathlib import Path

from rosbags.highlevel import AnyReader


def main():
    parser = argparse.ArgumentParser(description="Inspect a ROS 2 bag directory")
    parser.add_argument("bag", type=Path, help="Directory containing metadata.yaml and .db3 files")
    args = parser.parse_args()
    with AnyReader([args.bag]) as reader:
        for connection in reader.connections:
            print(connection.topic, connection.msgtype, connection.msgcount)
        # TODO: Deserialize messages and align images/actions for a LeRobot dataset.


if __name__ == "__main__":
    main()
