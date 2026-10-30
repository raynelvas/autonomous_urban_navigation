#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, String
import csv
import os
from pathlib import Path
import time

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        # Registration status
        self.is_logging_enabled = False
        self.current_log_path = None
        self.csv_writer = None
        self.csv_file = None
        self.node_name = "raw_sensors"  # Name to identify the file
        
        # Data buffer 
        self.data_buffer = []
        self.buffer_size = 10  # Number of messages before writing to disk
        self.last_flush_time = time.time()
        
        # Sensor data subscriptions
        self.create_subscription(
            Odometry, '/odom/unfiltered', self.odom_cb, 10)
        
        self.create_subscription(
            Imu, '/imu/unfiltered', self.imu_cb, 10)
            
        # Subscriptions to manager signals
        self.create_subscription(
            Bool, '/logging_enabled', self.logging_enabled_cb, 10)
            
        self.create_subscription(
            String, '/current_log_path', self.log_path_cb, 10)
        
        # CSV header
        self.header = [
            'stamp', 'topic', 'x', 'y', 'z',
            'qx', 'qy', 'qz', 'qw',
            'vx', 'vy', 'vz',  # linear velocities
            'wx', 'wy', 'wz',  # angular velocities
            'ax', 'ay', 'az',  # linear accelerations
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
            if self.is_logging_enabled and not self.csv_file:
                self._start_logging()

    def _start_logging(self):
        """Starts logging by creating the CSV file"""
        if not self.current_log_path:
            return
            
        try:
            # Create directory if it doesn't exist
            log_dir = Path(self.current_log_path)
            log_dir.mkdir(parents=True, exist_ok=True)
            
            # Specific file name for this node
            filename = log_dir / f"{self.node_name}.csv"
            
            # Open CSV file for writing
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Write header
            self.csv_writer.writerow(self.header)
            self.csv_file.flush()  # Force immediate write
            
            # Clear buffer if it had pending data
            self.data_buffer.clear()
        except Exception as e:
            self.csv_file = None
            self.csv_writer = None

    def _stop_logging(self):
        """Stops logging and closes the file"""
        # Write any pending data in the buffer
        self._flush_buffer()
        
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception as e:
                pass

        # Reset variables
        self.csv_file = None
        self.csv_writer = None
        self.data_buffer.clear()

    def _flush_buffer(self):
        """Writes the data from the buffer to the file"""
        if self.csv_writer and self.data_buffer:
            try:
                self.csv_writer.writerows(self.data_buffer)
                self.csv_file.flush()
                self.data_buffer.clear()
                self.last_flush_time = time.time()
            except Exception as e:
                pass

    def _write_data(self, row_data):
        """Writes a row of data """
        if not self.is_logging_enabled or not self.csv_writer:
            return
            
        self.data_buffer.append(row_data)
        
        # Write to disk if the buffer is full or too much time has passed
        if (len(self.data_buffer) >= self.buffer_size or 
            time.time() - self.last_flush_time > 1.0):  
            self._flush_buffer()

    def odom_cb(self, msg):
        """Callback for odometry data"""
        if not self.is_logging_enabled:
            return
            
        row = [
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'odom',
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z, 
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
            None, None, None  # There is no acceleration in odometry
        ]
        self._write_data(row)

    def imu_cb(self, msg):
        """Callback for IMU data"""
        if not self.is_logging_enabled:
            return
            
        # Extract angular and linear acceleration from IMU
        row = [
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'imu',
            None, None, None,  # There is no position
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            None, None, None,  # There is no linear velocity
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        self._write_data(row)

    def destroy_node(self):
        """Cleanup when destroying the node"""
        self._stop_logging()
        super().destroy_node()

def main():
    rclpy.init()
    node = DataRecorder()

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