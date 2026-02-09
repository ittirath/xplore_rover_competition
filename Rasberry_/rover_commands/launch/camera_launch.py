import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera_node',
            parameters=[{'video_device': '/dev/video0'}],
            output='screen'
        ),
        """
        # Image viewer
        # Node(
        #     package='rqt_image_view',
        #     executable='rqt_image_view',
        #     name='image_view',
        #     output='screen'
        # )
    ])
