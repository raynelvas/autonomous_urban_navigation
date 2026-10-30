#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import csv
import os
from pathlib import Path
import time
import numpy as np

class EKFRecorder(Node):
    def __init__(self):
        super().__init__('ekf_recorder')

        # Registration status
        self.is_logging_enabled = False
        self.current_log_path = None
        self.csv_writer_local = None
        self.csv_file_local = None
        self.node_name = "ekf_filters"
        
        # Data buffer
        self.local_buffer = []
        self.buffer_size = 10
        self.last_flush_time = time.time()
        
        # Subscriptions to EKF data
        self.create_subscription(
            Odometry, '/odometry/local', self.local_ekf_cb, 10)
            
        # Subscriptions to manager signals
        self.create_subscription(
            Bool, '/logging_enabled', self.logging_enabled_cb, 10)
            
        self.create_subscription(
            String, '/current_log_path', self.log_path_cb, 10)
        
        # CSV header
        self.local_header = [
            'stamp', 'frame_id', 'child_frame_id',
            'position_x', 'position_y', 'position_z',
            'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
            'velocity_x', 'velocity_y', 'velocity_z',
            'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
            # Position covariances (6x6 matrix - 36 elements)
            'pose_covariance_00', 'pose_covariance_01', 'pose_covariance_02', 
            'pose_covariance_03', 'pose_covariance_04', 'pose_covariance_05',
            'pose_covariance_06', 'pose_covariance_07', 'pose_covariance_08',
            'pose_covariance_09', 'pose_covariance_10', 'pose_covariance_11',
            'pose_covariance_12', 'pose_covariance_13', 'pose_covariance_14',
            'pose_covariance_15', 'pose_covariance_16', 'pose_covariance_17',
            'pose_covariance_18', 'pose_covariance_19', 'pose_covariance_20',
            'pose_covariance_21', 'pose_covariance_22', 'pose_covariance_23',
            'pose_covariance_24', 'pose_covariance_25', 'pose_covariance_26',
            'pose_covariance_27', 'pose_covariance_28', 'pose_covariance_29',
            'pose_covariance_30', 'pose_covariance_31', 'pose_covariance_32',
            'pose_covariance_33', 'pose_covariance_34', 'pose_covariance_35',
            # Velocity covariances (6x6 matrix - 36 elements)
            'twist_covariance_00', 'twist_covariance_01', 'twist_covariance_02',
            'twist_covariance_03', 'twist_covariance_04', 'twist_covariance_05',
            'twist_covariance_06', 'twist_covariance_07', 'twist_covariance_08',
            'twist_covariance_09', 'twist_covariance_10', 'twist_covariance_11',
            'twist_covariance_12', 'twist_covariance_13', 'twist_covariance_14',
            'twist_covariance_15', 'twist_covariance_16', 'twist_covariance_17',
            'twist_covariance_18', 'twist_covariance_19', 'twist_covariance_20',
            'twist_covariance_21', 'twist_covariance_22', 'twist_covariance_23',
            'twist_covariance_24', 'twist_covariance_25', 'twist_covariance_26',
            'twist_covariance_27', 'twist_covariance_28', 'twist_covariance_29',
            'twist_covariance_30', 'twist_covariance_31', 'twist_covariance_32',
            'twist_covariance_33', 'twist_covariance_34', 'twist_covariance_35'
        ]
        

    def logging_enabled_cb(self, msg):
        """Callback to enable/disable logging"""
        if msg.data != self.is_logging_enabled:
            self.is_logging_enabled = msg.data
            
            if self.is_logging_enabled:
                if self.current_log_path:
                    self._start_logging()
                else:
                    pass
            else:
                self._stop_logging()

    def log_path_cb(self, msg):
        """Callback to receive the logging path"""
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            
            # If logging is enabled but we haven't opened the file yet
            if self.is_logging_enabled and (not self.csv_file_local or not self.csv_file_global):
                self._start_logging()

    def _calculate_uncertainty(self, covariance_matrix):
        """Calcula métricas de incertidumbre a partir de la matriz de covarianza"""
        if len(covariance_matrix) != 36:
            return 0.0, 0.0, 0.0
            
        # Convert to numpy array
        cov = np.array(covariance_matrix).reshape(6, 6)
        
        # Diagonal matrix
        diagonal = np.diag(cov)
        
        # Uncertainty in position (first 3 elements)
        position_uncertainty = np.sqrt(np.sum(diagonal[0:3]))
        
        # Uncertainty in orientation (elements 3-5)
        orientation_uncertainty = np.sqrt(np.sum(diagonal[3:6]))
        
        velocity_uncertainty = 0.0
        if len(covariance_matrix) == 36:
            pass
            
        return position_uncertainty, orientation_uncertainty, velocity_uncertainty

    def _start_logging(self):
        """Start the registry by creating the CSV files"""
        if not self.current_log_path:
            return
            
        try:
            # Create directory if it does not exist
            log_dir = Path(self.current_log_path)
            log_dir.mkdir(parents=True, exist_ok=True)
            
            # EKF-specific file names
            filename_local = log_dir / f"ekf_local.csv"
            
            # Open CSV files for writing
            self.csv_file_local = open(filename_local, 'w', newline='')
            self.csv_writer_local = csv.writer(self.csv_file_local)            
            
            # Write header
            self.csv_writer_local.writerow(self.local_header)
            self.csv_file_local.flush() # Force immediate write
            
            # Clear buffers
            self.local_buffer.clear()
            
        except Exception as e:
            self.csv_file_local = None
            self.csv_writer_local = None

    def _stop_logging(self):
        """Stops logging and closes the file"""
        # Write any pending data in the buffer
        self._flush_buffers()
        
        if self.csv_file_local:
            try:
                self.csv_file_local.close()
            except Exception as e:
                pass
        
        # Reset variables
        self.csv_file_local = None
        self.csv_writer_local = None
        self.local_buffer.clear()

    def _flush_buffers(self):
        """Writes the data from the buffer to the file"""
        current_time = time.time()
        if self.csv_writer_local and self.local_buffer:
            try:
                self.csv_writer_local.writerows(self.local_buffer)
                self.csv_file_local.flush()
                self.local_buffer.clear()
            except Exception as e:
                pass
        
        self.last_flush_time = current_time

    def _write_local_data(self, row_data):
        """Writes a row of data """
        if not self.is_logging_enabled or not self.csv_writer_local:
            return
            
        self.local_buffer.append(row_data)
        
        # Write to disk if the buffer is full or too much time has passed
        if (len(self.local_buffer) >= self.buffer_size or 
            time.time() - self.last_flush_time > 1.0):
            self._flush_buffers()

    def local_ekf_cb(self, msg):
        """Callback para datos EKF local"""
        if not self.is_logging_enabled:
            return
            
        # Extract timestamp
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Create row with all data and covariances
        row = [timestamp, msg.header.frame_id, msg.child_frame_id]
        
        # Position
        row.extend([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # Orientation (quaternion)
        row.extend([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        
        # Linear velocity
        row.extend([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # Angular velocity
        row.extend([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])
        
        # Pose covariance (36 elements)
        row.extend(list(msg.pose.covariance))
        
        # Twist covariance (36 elements)
        row.extend(list(msg.twist.covariance))
        
        self._write_local_data(row)

    def destroy_node(self):
        """Cleanup when destroying the node"""
        self._stop_logging()
        super().destroy_node()

def main():
    rclpy.init()
    node = EKFRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop_logging()
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass 

if __name__ == '__main__':
    main()