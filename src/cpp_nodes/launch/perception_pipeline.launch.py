import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    home_path = os.path.expanduser("~")
    global_params_path = os.path.join(
        home_path, 
        'autonomous_navigation', 'src', 'params'
    )

    # Paths to the specific YAML files
    speed_yaml = os.path.join(global_params_path, 'speed_table.yaml')

    # We define the TwinLite engine path
    twinlite_engine_path = os.path.join(global_params_path, 'twinlite_FAST_32bit_ORIN.engine')
    yolov11_engine_path = os.path.join(global_params_path, 'detect_yolo_512x288_ORIN.engine')
    camera_calibration_path = os.path.join(global_params_path, 'camera_calibration_512x288.json')
    class_thresholds_yaml = os.path.join(global_params_path, 'class_thresholds.yaml')  

    return LaunchDescription([
        # 1. CAMERA NODE (Capture and Resize)
        Node(
            package='cpp_nodes',
            executable='camera_node',
            name='camera_node',
            parameters=[{'device': 0}],
            output='screen'
        ),

        # 2. SEGMENTER NODE 
        Node(
            package='cpp_nodes',
            executable='segmenter_node',
            name='segmenter_node',
            parameters=[{
                'engine_path': twinlite_engine_path
            }],
            output='screen'
        ),       

         # 4. IPM NODE segmentation (Transformation to Meters / PointCloud2))
        Node(
            package='cpp_nodes',
            executable='segmenter_to_meter',
            name='ipm_segmentation',
            parameters=[
                {'min_distance': 0.0},
                {'max_distance': 30.0},
                {'config_path': camera_calibration_path}
            ],
            output='screen'
        ),  

         # 5. DETECT NODE (YOLOv11n Inference)
        Node(
            package='cpp_nodes',
            executable='detect_node',
            name='detect_node',
            parameters=[{
                'engine_path': yolov11_engine_path},
                class_thresholds_yaml
                ],
            output='screen'
        ),

        
         # 6. IPM NODE object (Transformation to Meters / PointCloud2)
        Node(
            package='cpp_nodes',
            executable='object_to_meter',
            name='ipm_object',
            parameters=[
                {'min_distance': 0.0},
                {'max_distance': 10.0},
                {'config_path': camera_calibration_path},
                speed_yaml    # La ruta del archivo va sola en la lista
            ],
            output='screen'
        ),  

        # 7. ESP32 CONTROL NODE (Communication with microcontroller)
        Node(
            package='cpp_nodes',
            executable='esp32_control',
            name='esp32_control',
            output='screen'
        ), 

    ])