"""Record observations, commands and mode transitions for imitation learning."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output", default_value="/data/bags/robobike"),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record", "--storage", "sqlite3",
                "--output", LaunchConfiguration("output"),
                "/camera/image_raw", "/joy", "/teleop/cmd_vel",
                "/policy/cmd_vel", "/policy/enable", "/cmd_vel", "/control/state",
                "/robobike/telemetry", "/robobike/drive_state", "/robobike/bridge/connected",
            ],
            output="screen",
        ),
    ])
