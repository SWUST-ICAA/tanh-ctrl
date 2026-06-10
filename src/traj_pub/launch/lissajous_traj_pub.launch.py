from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_root = Path(get_package_share_directory("traj_pub"))
    params_file = package_root / "config" / "lissajous_figure8.yaml"

    return LaunchDescription(
        [
            Node(
                package="traj_pub",
                executable="lissajous_traj_pub",
                name="lissajous_traj_pub",
                output="screen",
                parameters=[str(params_file)],
            )
        ]
    )
