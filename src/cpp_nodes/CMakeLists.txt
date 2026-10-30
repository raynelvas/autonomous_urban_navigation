cmake_minimum_required(VERSION 3.8)
project(cpp_nodes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Buscar dependencias de ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 2. Buscar dependencias Externas
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# 3. Buscar TensorRT y CUDA (Rutas específicas de Jetson)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h HINTS /usr/include/aarch64-linux-gnu /usr/local/cuda/include)
find_library(TENSORRT_LIBRARY nvinfer HINTS /usr/lib/aarch64-linux-gnu /usr/local/cuda/lib64)
# Agregamos la librería de plugins de TensorRT (vital para YOLOv11)
find_library(TENSORRT_PLUGIN_LIBRARY nvinfer_plugin HINTS /usr/lib/aarch64-linux-gnu /usr/local/cuda/lib64)

find_package(CUDA REQUIRED)

include_directories(
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  /usr/local/cuda/include
)

# Esto ayuda a encontrar cudart y otras librerías de CUDA
link_directories(/usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu)

# 4. Definición de Ejecutables
add_executable(camera_node src/camera/camera.cpp)
add_executable(segmenter_node src/camera/segmenter_twin.cpp)
add_executable(detect_node src/camera/detect.cpp) 
add_executable(segmenter_to_meter src/camera/segmenter_to_meter.cpp)
add_executable(object_to_meter src/camera/object_to_meter.cpp)
add_executable(esp32_control src/esp32.cpp)

# 5. Linkeo de dependencias (Ament maneja el grueso)
ament_target_dependencies(camera_node rclcpp sensor_msgs cv_bridge OpenCV)
ament_target_dependencies(segmenter_node rclcpp sensor_msgs cv_bridge custom_interfaces OpenCV)
ament_target_dependencies(detect_node rclcpp sensor_msgs cv_bridge custom_interfaces OpenCV) 
ament_target_dependencies(segmenter_to_meter rclcpp sensor_msgs custom_interfaces)
ament_target_dependencies(object_to_meter rclcpp sensor_msgs custom_interfaces)
ament_target_dependencies(esp32_control rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs tf2 tf2_geometry_msgs Boost)

# 6. Linkeo Manual de Librerías no-ROS
# Para el segmentador (TensorRT + CUDA)
target_link_libraries(segmenter_node
    ${TENSORRT_LIBRARY}
    ${CUDA_LIBRARIES}
    cudart
)

# PARA EL DETECTOR (YOLOv11 con Plugins)
target_link_libraries(detect_node
    ${TENSORRT_LIBRARY}
    ${TENSORRT_PLUGIN_LIBRARY} # Requerido para YOLOv11
    ${CUDA_LIBRARIES}
    cudart
)

# Para IPM (JSON)
target_link_libraries(segmenter_to_meter nlohmann_json::nlohmann_json)
target_include_directories(segmenter_to_meter PRIVATE ${EIGEN3_INCLUDE_DIRS})

# Para IPM (JSON)
target_link_libraries(object_to_meter nlohmann_json::nlohmann_json)
target_include_directories(object_to_meter PRIVATE ${EIGEN3_INCLUDE_DIRS})

# 7. Instalación
install(TARGETS
  camera_node segmenter_node detect_node segmenter_to_meter object_to_meter esp32_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()