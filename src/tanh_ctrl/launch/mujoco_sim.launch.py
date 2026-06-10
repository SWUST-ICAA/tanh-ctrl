from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulator_share = Path(get_package_share_directory("uav_simulator"))
    controller_share = Path(get_package_share_directory("tanh_ctrl"))
    traj_share = Path(get_package_share_directory("traj_pub"))

    simulator_params = simulator_share / "config" / "uav_simulator.yaml"
    controller_params = controller_share / "config" / "tanh_ctrl.yaml"
    traj_params = traj_share / "config" / "lissajous_figure8.yaml"
    model_path = simulator_share / "models" / "real_x2" / "scene.xml"

    use_viewer_arg = DeclareLaunchArgument("use_viewer", default_value="true")
    use_param_gui_arg = DeclareLaunchArgument("use_param_gui", default_value="true")

    return LaunchDescription(
        [
            use_viewer_arg,
            use_param_gui_arg,
            Node(
                package="uav_simulator",
                executable="mujoco_px4_bridge",
                name="uav_simulator",
                output="screen",
                parameters=[
                    str(simulator_params),
                    {
                        "model_path": str(model_path),
                        "use_viewer": LaunchConfiguration("use_viewer"),
                    },
                ],
            ),
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
