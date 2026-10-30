
---

# Autonomous Navigation for a Differential Drive Mobile Robot

Autonomous navigation and control system developed in **ROS 2 Humble** for a differential-drive mobile robot. The project integrates a real-time perception pipeline based on lightweight neural networks, state estimation using an **Extended Kalman Filter (EKF)**, and trajectory control using **Non-linear Model Predictive Control (NMPC)**.

<p align="center">
  <img src="multimedia/nexus.gif" alt="nexus" width="700"/>
</p>

---

## Authors

* **Luis Raynel Vasquez Quishpe**  
  *Email:* luchitov2001@gmail.com

<p align="center">
  <img src="multimedia/NEXUS.jpeg" alt="NEXUS" width="300"/>
</p>

---

## Repository Structure

### Node Distribution in ROS 2

```text
src/
├── cpp_nodes/             # Camera communication
│                          # Neural network execution
│                          # IPM transformation
│                          # Control ESP32 communication
│
├── custom_interfaces/     # Custom messages
│
├── filter_ekf/            # Robot linear and angular velocity filtering
│
├── navigation_system/     # NMPC control algorithm
│
├── params/                # YAML configuration files
│                          # Neural network files
│
├── perception_stack/      # Geometric lane estimation
│                          # EKF filtering of the lane model
│                          # Image capture
│                          # Visual debugging
│
├── robot_description/     # URDF/Xacro models and TF configurations
│                          # RViz2 configuration
│
├── robot_drivers/         # WebRTC and WebSocket communication
│
├── saves/                 # Nodes for real-time system data logging
│                          # Three nodes for data and image playback
│
├── system_management/     # Nodes for real-time system monitoring
│                          # Communication with the safety ESP32
└──                        # Full system launch file 
```

### Additional Folder

```text
ESP32/
├── libraries                                # Libraries required for the Arduino code
├── Robot_Odometry_Simplify_Serial/          # Arduino code for the control ESP32 with MPU9250
├── Robot_Odometry_Simplify_Serial_MPU6500/  # Arduino code for the control ESP32 with MPU6500
├── Robot_Safety_Simplify_Serial/            # Arduino code for the safety ESP32
├── test_codes/                              # Additional code for testing components individually
└──                                          # Code for PID configuration
```

---

## Prerequisites

* **OS:** Ubuntu 22.04 LTS

* **ROS 2:** Humble

* **C++ / Python dependencies:**

  * OpenCV

  * TensorRT & CUDA (for GPU-accelerated inference)

---

## Usage / Execution

To start all system nodes on both the Jetson and the computer (perception, estimation, and control):

```bash
ros2 launch system_management management_all_system.launch.py
```

### Playing Back Data on the Computer

To play back pre-recorded data on the computer:

* Display images one by one:

```bash
ros2 run saves image_publisher
```

* Publish images, lane changes, and manual mode at a specific frequency:

```bash
ros2 run saves image_mode
```

* Publish sensor data, images, and operating modes in real time:

```bash
ros2 run saves data_player
```

For the three nodes, it is necessary to provide the location of the data to be replayed:

```bash
ros2 topic pub /replay_log_path std_msgs/String "data: '/home/raynel/autonomous_navigation/src/saves/data_logs/universidad2/ruta47'" --once
```

### Saving Generated Debug Images to the Computer

Configure the location within the node and execute it:

```bash
ros2 run perception_stack image_saver_debug
```

### Visualization in RViz2

```bash
ros2 launch robot_description rviz2.launch.py
```

---

## Neural Networks

The trained models are provided in both their original format and ONNX format.

- 📁 [Google Drive of neural networks](https://drive.google.com/drive/folders/1ZxTVijdh90kl1DD3gjYYYsyJzCp4-xVn)

Starting from the ONNX model, it is necessary to generate a TensorRT model and place the resulting file in the `params` folder.

```bash
trtexec --onnx=twinlite_512x288_lines_v2.onnx --saveEngine=twinlite_v2_RTX4060.engine --fp16 --builderOptimizationLevel=5
```

### YOLOv11n Detection Network

<p align="center">
  <img src="multimedia/detection.jpg" alt="detection" width="400"/>
</p>

### [TwinLiteNet](https://github.com/chequanghuy/TwinLiteNet.git) Lane Segmentation Network

<p align="center">
  <img src="multimedia/segmentation.jpg" alt="segmentation" width="400"/>
</p>

---
