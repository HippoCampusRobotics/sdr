from hippo_common.launch_helper import LaunchArgsDict
from launch_ros.actions import Node

from launch import LaunchDescription


def create_sdr_node():
    LaunchArgsDict()
    return Node(
        package='sdr',
        executable='sdr_node.py',
        parameters=[],
        output='screen',
        emulate_tty=True,
    )


def generate_launch_description():
    launch_description = LaunchDescription()

    action = create_sdr_node()

    launch_description.add_action(action)

    return launch_description
