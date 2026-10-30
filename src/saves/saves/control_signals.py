#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
from pathlib import Path
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from custom_interfaces.msg import LaneModel
from std_msgs.msg import Bool, String, Float32
from diagnostic_msgs.msg import DiagnosticStatus

class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger')
        
        self.is_logging_enabled = False
        self.current_log_path = None
        
        # Standalone CSV files
        self.loggers = {
            'lane':   {'file': None, 'writer': None, 'filename': 'lane_data.csv'},
            'odom':   {'file': None, 'writer': None, 'filename': 'odom_data.csv'},
            'cmd':    {'file': None, 'writer': None, 'filename': 'control_data.csv'},
            'manual': {'file': None, 'writer': None, 'filename': 'control_manual.csv'}
        }

        # Context variables
        self.emergency = False
        self.safe_stop = False
        self.alpha_vision = 1.0
        self.active_vision = False
        self.global_level = 0
        self.is_manual = False
        self.change_left = False
        self.change_right = False

        # --- Data Subscribers ---
        self.create_subscription(LaneModel, '/lane/model_filtered', self.cb_lane_model, 10)
        self.create_subscription(Odometry, '/odometry/local', self.cb_odometry, 10) 
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.create_subscription(Twist, '/cmd_vel_manual', self.cb_manual_twist, 10)

        # --- Status Subscribers ---
        self.create_subscription(Bool, '/emergency', lambda msg: setattr(self, 'emergency', msg.data), 10)
        self.create_subscription(Bool, '/safe_stop', lambda msg: setattr(self, 'safe_stop', msg.data), 10)
        self.create_subscription(Float32, '/alpha/vision', lambda msg: setattr(self, 'alpha_vision', msg.data), 10)
        self.create_subscription(Bool, '/active/vision', lambda msg: setattr(self, 'active_vision', msg.data), 10)
        self.create_subscription(Bool, '/manual', lambda msg: setattr(self, 'is_manual', msg.data), 10)
        self.create_subscription(DiagnosticStatus, '/global_status', 
            lambda msg: setattr(self, 'global_level', int.from_bytes(msg.level, 'little')), 10)
        
        # --- Data Subscribers for Lane Changes ---
        self.create_subscription(Bool, '/change/left', lambda msg: setattr(self, 'change_left', msg.data), 10)
        self.create_subscription(Bool, '/change/right', lambda msg: setattr(self, 'change_right', msg.data), 10)

        self.create_subscription(Bool, '/logging_enabled', self.logging_enabled_cb, 10)
        self.create_subscription(String, '/current_log_path', self.log_path_cb, 10)

    def _start_logging(self):
        if not self.current_log_path: return
        
        base_path = Path(self.current_log_path)
        node_dir = base_path / "robot_control" 

        try:
            node_dir.mkdir(parents=True, exist_ok=True)
            
            for key in self.loggers:
                file_path = node_dir / self.loggers[key]['filename']
                f = open(file_path, 'w', newline='')
                w = csv.writer(f)
                
                # Headers
                if key == 'lane':
                    w.writerow(['timestamp', 'lane_d_lat', 'lane_yaw', 'lane_curvature', 'lane_confidence', 'lane_width', 
                                'emergency', 'safe_stop', 'alpha_vision', 'active_vision', 'global_level', 'is_manual', 'change_left', 'change_right'])
                elif key == 'odom':
                    w.writerow(['timestamp', 'odom_linear_x', 'odom_angular_z'])
                elif key == 'cmd':
                    w.writerow(['timestamp', 'cmd_vel_linear_x', 'cmd_vel_angular_z'])
                elif key == 'manual':
                    w.writerow(['timestamp', 'manual_lin_x', 'manual_ang_z'])
                
                self.loggers[key].update({'file': f, 'writer': w})

        except Exception as e:
            self.get_logger().error(f"Error creating files: {e}")

    # ================= DIRECT WRITE CALLBACKS =================
    def cb_lane_model(self, msg):
        if not self.is_logging_enabled or not self.loggers['lane']['writer']: return
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # We load the additional state parameters
        row = [
            f"{ts:.6f}", msg.d_lat, msg.yaw, msg.curvature, msg.confidence, msg.lane_width, 
            int(self.emergency), int(self.safe_stop), f"{self.alpha_vision:.2f}", 
            int(self.active_vision), self.global_level, int(self.is_manual), int(self.change_left), int(self.change_right)
        ]
        self.loggers['lane']['writer'].writerow(row)

    def cb_odometry(self, msg):
        if not self.is_logging_enabled or not self.loggers['odom']['writer']: return
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        row = [f"{ts:.6f}", msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        self.loggers['odom']['writer'].writerow(row)

    def cb_cmd_vel(self, msg):
        if not self.is_logging_enabled or not self.loggers['cmd']['writer']: return
        ts = self.get_clock().now().nanoseconds * 1e-9
        row = [f"{ts:.6f}", msg.linear.x, msg.angular.z]
        self.loggers['cmd']['writer'].writerow(row)
    
    def cb_manual_twist(self, msg):
        if not self.is_logging_enabled or not self.loggers['manual']['writer']: return
        ts = self.get_clock().now().nanoseconds * 1e-9
        row = [f"{ts:.6f}", msg.linear.x, msg.angular.z]
        self.loggers['manual']['writer'].writerow(row)

    # ================= MANAGEMENT AND CLEANUP =================
    def logging_enabled_cb(self, msg):
        if msg.data != self.is_logging_enabled:
            self.is_logging_enabled = msg.data
            self._start_logging() if self.is_logging_enabled else self._stop_logging()

    def log_path_cb(self, msg):
        self.current_log_path = msg.data

    def _stop_logging(self):
        for key in self.loggers:
            if self.loggers[key]['file']:
                self.loggers[key]['file'].close()
                self.loggers[key].update({'file': None, 'writer': None})

    def destroy_node(self):
        self._stop_logging()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass