from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = Path(get_package_share_directory("uav_simulator"))
    params_file = share_dir / "config" / "uav_simulator.yaml"
    model_path = share_dir / "models" / "real_x2" / "scene.xml"

    use_viewer_arg = DeclareLaunchArgument("use_viewer", default_value="true")

    return LaunchDescription(
        [
            use_viewer_arg,
            Node(
                package="uav_simulator",
                executable="mujoco_px4_bridge",
                name="uav_simulator",
                output="screen",
                parameters=[
                    str(params_file),
                    {
                        "model_path": str(model_path),
                        "use_viewer": LaunchConfiguration("use_viewer"),
                    },
                ],
            ),
        ]
    )
