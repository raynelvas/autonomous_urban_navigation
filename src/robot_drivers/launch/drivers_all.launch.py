from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch WebSocket Bridge
        Node(
            package='robot_drivers',
            executable='websocket_bridge',
            output='screen',
            name='websocket'
        ),

        # Launch the WebRTC node
        Node(
            package='robot_drivers',
            executable='webrtc_streamer',
            output='screen',
            name='webrtc_streamer'
        ),                   
    ])

