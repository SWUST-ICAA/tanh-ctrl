from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    params_file = package_root / "config" / "tanh_ctrl.yaml"

    return LaunchDescription(
        [
            Node(
                package="tanh_ctrl",
                executable="tanh_ctrl_node",
                name="tanh_ctrl",
                output="screen",
                parameters=[str(params_file)],
            )
        ]
    )
