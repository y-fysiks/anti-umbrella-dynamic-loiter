import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config_path = os.path.join(
        get_package_share_directory("dronepid"), "config.yaml"
    )

    dronepid_node = Node(
        package="dronepid",
        executable="dronepid",
        name="dronepid_node",
        parameters=[config_path],
    )

    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera_node"
    )

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_node",
        output="screen",
        emulate_tty=True,
        parameters=[os.path.join(get_package_share_directory("apriltag_ros"), "cfg", "tags_36h11.yaml")],
        remappings=[
            ("image_rect", "/image_raw"),
            ("camera_info", "/camera_info"),
        ],
    )
    

    ld.add_action(dronepid_node)
    ld.add_action(camera_node)
    ld.add_action(apriltag_node)
    return ld
