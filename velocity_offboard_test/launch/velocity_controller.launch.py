from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    controller_node = Node(
        package='velocity_offboard_test',
        executable='velocity_controller',
        parameters=[{'use_sim_time': True}]
    )

    return [controller_node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
