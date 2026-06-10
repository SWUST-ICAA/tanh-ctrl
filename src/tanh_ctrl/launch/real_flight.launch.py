from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_share = Path(get_package_share_directory("tanh_ctrl"))
    traj_share = Path(get_package_share_directory("traj_pub"))

    controller_params = controller_share / "config" / "tanh_ctrl.yaml"
    traj_params = traj_share / "config" / "lissajous_figure8.yaml"

    use_traj_pub_arg = DeclareLaunchArgument("use_traj_pub", default_value="true")
    use_param_gui_arg = DeclareLaunchArgument("use_param_gui", default_value="false")

    return LaunchDescription(
        [
            use_traj_pub_arg,
            use_param_gui_arg,
            Node(
                package="tanh_ctrl",
                executable="tanh_ctrl_node",
                name="tanh_ctrl",
                output="screen",
                parameters=[str(controller_params)],
            ),
            Node(
                package="traj_pub",
                executable="lissajous_traj_pub",
                name="lissajous_traj_pub",
                output="screen",
                parameters=[str(traj_params)],
                condition=IfCondition(LaunchConfiguration("use_traj_pub")),
            ),
            Node(
                package="tanh_ctrl",
                executable="tanh_ctrl_param_gui",
                name="tanh_ctrl_param_gui",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_param_gui")),
            ),
        ]
    )
