from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    mecanum_wheels = LaunchConfiguration("mecanum_wheels").perform(context)

    bringup_share_dir = get_package_share_directory("leo_bringup")
    diff_drive_params = os.path.join(bringup_share_dir, "config", "bldc_manager_diff_drive_params.yaml")
    mecanum_params   = os.path.join(bringup_share_dir, "config", "bldc_manager_mecanum_params.yaml")
    common_params    = os.path.join(bringup_share_dir, "config", "bldc_manager_common_params.yaml")

    controller_params_file = mecanum_params if mecanum_wheels.lower() == "true" else diff_drive_params

    bldc_manager_node = Node(
        package="leo_bldc",
        executable="bldc_manager",
        parameters=[
            {"tf_frame_prefix": LaunchConfiguration("tf_frame_prefix")},
            {"mecanum_wheels": LaunchConfiguration("mecanum_wheels")},
            common_params,
            controller_params_file
        ],
        output="screen"
    )

    return [TimerAction(period=3.0, actions=[bldc_manager_node])]

def generate_launch_description():
    mecanum_wheels_arg = DeclareLaunchArgument(
        "mecanum_wheels",
        default_value="false",
        description="Flag specifying wheel types of the robot"
    )

    tf_frame_prefix_arg = DeclareLaunchArgument(
        "tf_frame_prefix",
        default_value="",
        description="The prefix added to each published TF frame id"
    )

    return LaunchDescription([
        mecanum_wheels_arg,
        tf_frame_prefix_arg,
        OpaqueFunction(function=launch_setup)
    ])