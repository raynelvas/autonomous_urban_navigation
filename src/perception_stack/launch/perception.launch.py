from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    home_path = os.path.expanduser("~")
    global_params_path = os.path.join(
        home_path, 
        'autonomous_navigation', 'src', 'params'
    )
    
    camera_calibration_path = os.path.join(global_params_path, 'camera_calibration_512x288.json')
    save_directory_path = '/home/raynel/Documents/calibrate_camera'

    return LaunchDescription([

        # ========= Capture frame node ===================
        Node(
            package='perception_stack',
            executable='capture_frame',
            parameters=[{
                'save_directory': save_directory_path 
            }],
            output='screen',
        ),
        
        # ===============PAPER =============
        Node(
            package='perception_stack',
            executable='lane_model_estimator',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='lane_model_tracking',
            output='screen',
        ),

        # ============ VISUALIZADORES =============
        Node(
            package='perception_stack',
            executable='visualizer_camera_front',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='inverse_IPM',
            parameters=[{
                'config_path': camera_calibration_path
            }],
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='clusters_inverse_projection',
            parameters=[{
                'config_path': camera_calibration_path
            }],
            output='screen',
        ), 
    ])