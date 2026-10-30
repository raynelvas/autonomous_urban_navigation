cmake_minimum_required(VERSION 3.8)
project(cpp_nodes)

# 1. Estándar C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. Buscar dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui)
find_package(custom_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED) 
find_package(Boost REQUIRED COMPONENTS system)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# 3. Buscar CUDA y TensorRT con rutas estrictas
find_package(CUDAToolkit REQUIRED)

# Buscamos TensorRT y sus componentes (Crucial: Plugin Library)
find_library(NVINFER_LIB nvinfer 
  PATHS ${CUDAToolkit_LIBRARY_DIR} /usr/local/cuda/lib64 /usr/lib/x86_64-linux-gnu
  NO_DEFAULT_PATH
)

find_library(NVINFER_PLUGIN_LIB nvinfer_plugin 
  PATHS ${CUDAToolkit_LIBRARY_DIR} /usr/local/cuda/lib64 /usr/lib/x86_64-linux-gnu
  NO_DEFAULT_PATH
)

find_library(NVONNX_PARSER_LIB nvonnxparser 
  PATHS ${CUDAToolkit_LIBRARY_DIR} /usr/local/cuda/lib64 /usr/lib/x86_64-linux-gnu
  NO_DEFAULT_PATH
)

# Agrupamos las librerías críticas incluyendo PLUGINS
set(TENSORRT_LIBS ${NVINFER_LIB} ${NVINFER_PLUGIN_LIB} ${NVONNX_PARSER_LIB})

# Mensajes de depuración
message(STATUS ">>> TensorRT Principal: ${NVINFER_LIB}")
message(STATUS ">>> TensorRT Plugins: ${NVINFER_PLUGIN_LIB}")

# 4. Ejecutables
add_executable(camera_node src/camera/camera.cpp)
add_executable(segmenter_twin_node src/camera/segmenter_twin.cpp)
add_executable(detect_node src/camera/detect.cpp)
add_executable(segmenter_to_meter src/camera/segmenter_to_meter.cpp)
add_executable(esp32_control src/esp32.cpp)
add_executable(object_to_meter src/camera/object_to_meter.cpp)

# 5. Dependencias de ROS (ament_target_dependencies)
ament_target_dependencies(camera_node rclcpp sensor_msgs cv_bridge OpenCV)
ament_target_dependencies(segmenter_to_meter rclcpp sensor_msgs custom_interfaces)
ament_target_dependencies(esp32_control rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs tf2 tf2_geometry_msgs Boost)
ament_target_dependencies(object_to_meter rclcpp sensor_msgs custom_interfaces)

ament_target_dependencies(segmenter_twin_node
  rclcpp
  sensor_msgs
  cv_bridge
  custom_interfaces
  OpenCV
)

# Detectr_node - dependencias base de ROS
ament_target_dependencies(detect_node
  rclcpp
  sensor_msgs
  cv_bridge
  custom_interfaces
  OpenCV
)

# 6. CONFIGURACIÓN ESPECÍFICA PARA EL SEGMENTADOR (TensorRT + CUDA)
target_include_directories(segmenter_twin_node PRIVATE 
  ${CUDAToolkit_INCLUDE_DIRS}
  /usr/include/x86_64-linux-gnu
)

target_link_libraries(segmenter_twin_node
  ${TENSORRT_LIBS}
  CUDA::cudart
  ${OpenCV_LIBRARIES}
  dl
  pthread
)


# 6. CONFIGURACIÓN ESPECÍFICA PARA EL SEGMENTADOR (TensorRT + CUDA)
target_include_directories(detect_node PRIVATE 
  ${CUDAToolkit_INCLUDE_DIRS}
  /usr/include/x86_64-linux-gnu
)

# Enlace de librerías: Agregamos TENSORRT_LIBS (que ya tiene los plugins)
target_link_libraries(detect_node
  ${TENSORRT_LIBS}
  CUDA::cudart
  ${OpenCV_LIBRARIES}
  dl
  pthread
)

# RPATH para que encuentre las librerías en tiempo de ejecución
set_target_properties(detect_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
  INSTALL_RPATH "${CUDAToolkit_LIBRARY_DIR}:/usr/local/cuda/lib64:/usr/lib/x86_64-linux-gnu"
)

# 7. Configuración para otros nodos
target_include_directories(segmenter_to_meter PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(segmenter_to_meter nlohmann_json::nlohmann_json)

target_link_libraries(camera_node ${OpenCV_LIBRARIES})
target_include_directories(object_to_meter PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(object_to_meter nlohmann_json::nlohmann_json)

# 8. Instalación
install(TARGETS
  camera_node
  segmenter_twin_node
  detect_node
  segmenter_to_meter
  esp32_control
  object_to_meter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()