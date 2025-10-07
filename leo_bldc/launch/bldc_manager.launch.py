from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bldc_manager_node = Node(
        package="leo_bldc",
        executable="bldc_manager",
        parameters=[
            {"tf_frame_prefix": LaunchConfiguration("tf_frame_prefix")},
            {"mecanum_wheels": LaunchConfiguration("mecanum_wheels")},
            LaunchConfiguration("params_file")
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

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Full path to the YAML parameter file for bldc_manager"
    )

    return LaunchDescription([
        mecanum_wheels_arg,
        tf_frame_prefix_arg,
        params_file_arg,
        OpaqueFunction(function=launch_setup)
    ])