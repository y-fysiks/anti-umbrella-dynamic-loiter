# ros2 run v4l2_camera v4l2_camera_node

# ros2 run apriltag_ros apriltag_node --ros-args     -r image_rect:=/image_raw     -r camera_info:=/camera_info    
# --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    