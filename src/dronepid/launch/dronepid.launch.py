import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config_path = os.path.join(
        get_package_share_directory("dronepid"), "config", "config.yaml"
    )

    dronepid_node = Node(
        package="dronepid",
        executable="dronepid",
        name="dronepid_node",
        output="screen",
        emulate_tty=True,
        parameters=[config_path],
    )

    ld.add_action(dronepid_node)
    return ld
