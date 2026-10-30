#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import csv
import os
from pathlib import Path
import time
from datetime import datetime

class SystemHealthRecorder(Node):
    def __init__(self):
        super().__init__('system_health_recorder')
        
        # Registration status
        self.is_logging_enabled = False
        self.current_log_path = None
        self.node_name = "system_health"
        
        # Files
        self.csv_file = None
        self.csv_writer = None
        self.current_session_dir = None
        
        # Buffer para datos
        self.data_buffer = []
        self.buffer_size = 10
        self.last_flush_time = time.time()
        
        # Dictionary to store last values
        self.current_values = {
            'timestamp': None,
            # Temperatures
            'cpu_temp': None,
            'gpu_temp': None,
            'disk_temp': None,
            # Battery
            'battery_percent': None,
            'battery_plugged': None,
            # Resource usage
            'cpu_usage': None,
            'gpu_usage': None,
            'ram_usage': None,
            'ram_used_gb': None,
            'ram_total_gb': None,
            # Disk
            'disk_usage': None,
            'disk_free_gb': None,
            # System
            'uptime_hours': None,
            'fan_rpm': None,
            # General condition
            'system_level': None,
            # Individual level
            'cpu_temp_level': None,
            'gpu_temp_level': None,
            'battery_level': None,
            'ram_level': None,
            'cpu_usage_level': None,
            'disk_temp_level': None
        }
        
        # Subscriptions to all health topics in the system
        topics = [
            ('/status/cpu_temperature', self.cpu_temp_cb),
            ('/status/gpu_temperature', self.gpu_temp_cb),
            ('/status/battery_laptop', self.battery_cb),
            ('/status/ram', self.ram_cb),
            ('/status/cpu_usage', self.cpu_usage_cb),
            ('/status/gpu_usage', self.gpu_usage_cb),
            ('/status/disk_temperature', self.disk_temp_cb),
            ('/status/uptime', self.uptime_cb),
            ('/status/system_summary', self.system_summary_cb)
        ]
        
        for topic, callback in topics:
            self.create_subscription(
                DiagnosticStatus, topic, callback, 10
            )
        
        # Subscriptions to manager signals
        self.create_subscription(Bool, '/logging_enabled', self.logging_enabled_cb, 10)
        self.create_subscription(String, '/current_log_path', self.log_path_cb, 10)
        
        # Timer to save data periodically
        self.save_timer = self.create_timer(1.0, self.save_health_data)  # 1 Hz

    def logging_enabled_cb(self, msg):
        """Callback to enable/disable logging"""
        if msg.data != self.is_logging_enabled:
            self.is_logging_enabled = msg.data
            
            if self.is_logging_enabled:
                if self.current_log_path:
                    self._start_logging()
            else:
                self._stop_logging()

    def log_path_cb(self, msg):
        """Callback to receive the logging path"""
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            
            if self.is_logging_enabled and not self.current_session_dir:
                self._start_logging()

    def _extract_value(self, values, key, default=None):
        """Extract a specific value from the KeyValue array"""
        for kv in values:
            if kv.key == key:
                return kv.value
        return default

    # Callbacks for each type of data
    def cpu_temp_cb(self, msg):
        self.current_values['timestamp'] = time.time()
        self.current_values['cpu_temp'] = float(self._extract_value(msg.values, 'temperature', '0.0'))
        self.current_values['cpu_temp_level'] = msg.level
        
    def gpu_temp_cb(self, msg):
        self.current_values['timestamp'] = time.time()
        self.current_values['gpu_temp'] = float(self._extract_value(msg.values, 'temperature', '0.0'))
        self.current_values['gpu_temp_level'] = msg.level
        
    def battery_cb(self, msg):
        self.current_values['timestamp'] = time.time()
        self.current_values['battery_percent'] = float(self._extract_value(msg.values, 'percentage', '100.0'))
        self.current_values['battery_plugged'] = self._extract_value(msg.values, 'plugged', 'yes') == 'yes'
        self.current_values['battery_level'] = msg.level
        
    def ram_cb(self, msg):
        self.current_values['timestamp'] = time.time()
        self.current_values['ram_usage'] = float(self._extract_value(msg.values, 'usage_percent', '0.0'))
        self.current_values['ram_used_gb'] = float(self._extract_value(msg.values, 'used_gb', '0.0'))
        self.current_values['ram_total_gb'] = float(self._extract_value(msg.values, 'total_gb', '0.0'))
        self.current_values['ram_level'] = msg.level
        
    def cpu_usage_cb(self, msg):
        self.current_values['timestamp'] = time.time()
        self.current_values['cpu_usage'] = float(self._extract_value(msg.values, 'usage_percent', '0.0'))
        self.current_values['cpu_usage_level'] = msg.level
        
    def gpu_usage_cb(self, msg):
        self.current_values['timestamp'] = time.time()
        self.current_values['gpu_usage'] = float(self._extract_value(msg.values, 'usage_percent', '0.0'))
        
    def disk_temp_cb(self, msg):
        self.current_values['timestamp'] = time.time()
        self.current_values['disk_temp'] = float(self._extract_value(msg.values, 'temperature', '0.0'))
        self.current_values['disk_temp_level'] = msg.level
        self.current_values['disk_usage'] = float(self._extract_value(msg.values, 'usage', '0.0').replace('%', ''))
        
    def uptime_cb(self, msg):
        self.current_values['timestamp'] = time.time()
        self.current_values['uptime_hours'] = float(self._extract_value(msg.values, 'uptime_hours', '0.0'))
        
    def system_summary_cb(self, msg):
        self.current_values['timestamp'] = time.time()
        self.current_values['system_level'] = msg.level
        
        # Extract additional values ​​from summary
        self.current_values['cpu_temp'] = float(self._extract_value(msg.values, 'cpu_temp', 
                                                                   str(self.current_values.get('cpu_temp', '0.0'))))
        self.current_values['gpu_temp'] = float(self._extract_value(msg.values, 'gpu_temp', 
                                                                   str(self.current_values.get('gpu_temp', '0.0'))))
        self.current_values['battery_percent'] = float(self._extract_value(msg.values, 'battery', 
                                                                          str(self.current_values.get('battery_percent', '100.0'))))
        self.current_values['cpu_usage'] = float(self._extract_value(msg.values, 'cpu_usage', 
                                                                    str(self.current_values.get('cpu_usage', '0.0'))))
        self.current_values['ram_usage'] = float(self._extract_value(msg.values, 'ram_usage', 
                                                                    str(self.current_values.get('ram_usage', '0.0'))))
        self.current_values['fan_rpm'] = float(self._extract_value(msg.values, 'fan_rpm', '0.0'))
        
    def _start_logging(self):
        """Start the registry by creating the CSV files"""
        if not self.current_log_path:
            return
            
        try:
            # Create directory if it does not exist
            log_dir = Path(self.current_log_path)
            log_dir.mkdir(parents=True, exist_ok=True)
            
            # Specific file name
            filename = log_dir / f"{self.node_name}.csv"
            
            # Open CSV file for writing
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # CSV Header
            header = [
                'timestamp',
                'cpu_temp_c', 'gpu_temp_c', 'disk_temp_c',
                'battery_percent', 'battery_plugged',
                'cpu_usage_percent', 'gpu_usage_percent', 'ram_usage_percent',
                'ram_used_gb', 'ram_total_gb',
                'disk_usage_percent', 'disk_free_gb',
                'uptime_hours', 'fan_rpm',
                # Level (0=OK, 1=WARN, 2=ERROR)
                'system_level', 'cpu_temp_level', 'gpu_temp_level',
                'battery_level', 'ram_level', 'cpu_usage_level', 'disk_temp_level',
                # Calculated Flags
                'any_warning', 'any_error', 'overheating_risk', 'battery_low'
            ]
            self.csv_writer.writerow(header)
            self.csv_file.flush()
            
            # Clear buffer
            self.data_buffer.clear()
            
            # Reset values
            self._reset_values()
            
        except Exception as e:
            self.csv_file = None
            self.csv_writer = None

    def _reset_values(self):
        """Reset all values ​​to None"""
        for key in self.current_values:
            self.current_values[key] = None

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

    def _calculate_flags(self):
        """Calculates flags derived from health data"""
        # Check if there are warnings
        any_warning = any([
            self.current_values.get('cpu_temp_level') == 1,
            self.current_values.get('gpu_temp_level') == 1,
            self.current_values.get('battery_level') == 1,
            self.current_values.get('ram_level') == 1,
            self.current_values.get('cpu_usage_level') == 1,
            self.current_values.get('disk_temp_level') == 1
        ])
        
        # Check for errors
        any_error = any([
            self.current_values.get('cpu_temp_level') == 2,
            self.current_values.get('gpu_temp_level') == 2,
            self.current_values.get('battery_level') == 2,
            self.current_values.get('ram_level') == 2,
            self.current_values.get('cpu_usage_level') == 2,
            self.current_values.get('disk_temp_level') == 2
        ])
        
        # Risk of overheating
        overheating_risk = any([
            self.current_values.get('cpu_temp') and self.current_values['cpu_temp'] > 85,
            self.current_values.get('gpu_temp') and self.current_values['gpu_temp'] > 85
        ])
        
        # Low battery
        battery_low = (
            self.current_values.get('battery_percent') and 
            self.current_values['battery_percent'] < 20 and
            not self.current_values.get('battery_plugged')
        )
        
        return any_warning, any_error, overheating_risk, battery_low

    def save_health_data(self):
        """Save health data periodically"""
        if not self.is_logging_enabled or not self.csv_writer:
            return
        
        # Only save if we have valid data
        has_data = any(
            self.current_values[key] is not None 
            for key in self.current_values 
            if key != 'timestamp'
        )
        
        if has_data and self.current_values['timestamp'] is not None:
            # Calculate flags
            any_warning, any_error, overheating_risk, battery_low = self._calculate_flags()
            
            # Prepare data row
            row = [
                self.current_values['timestamp'],
                # Temperature
                self.current_values['cpu_temp'] if self.current_values['cpu_temp'] is not None else '',
                self.current_values['gpu_temp'] if self.current_values['gpu_temp'] is not None else '',
                self.current_values['disk_temp'] if self.current_values['disk_temp'] is not None else '',
                # Battery
                self.current_values['battery_percent'] if self.current_values['battery_percent'] is not None else '',
                1 if self.current_values.get('battery_plugged') else 0 if self.current_values.get('battery_plugged') is not None else '',
                # Resource usage
                self.current_values['cpu_usage'] if self.current_values['cpu_usage'] is not None else '',
                self.current_values['gpu_usage'] if self.current_values['gpu_usage'] is not None else '',
                self.current_values['ram_usage'] if self.current_values['ram_usage'] is not None else '',
                self.current_values['ram_used_gb'] if self.current_values['ram_used_gb'] is not None else '',
                self.current_values['ram_total_gb'] if self.current_values['ram_total_gb'] is not None else '',
                # Disk
                self.current_values['disk_usage'] if self.current_values['disk_usage'] is not None else '',
                self.current_values['disk_free_gb'] if self.current_values.get('disk_free_gb') is not None else '',
                # System
                self.current_values['uptime_hours'] if self.current_values['uptime_hours'] is not None else '',
                self.current_values['fan_rpm'] if self.current_values.get('fan_rpm') is not None else '',
                # Levels
                self.current_values['system_level'] if self.current_values['system_level'] is not None else '',
                self.current_values['cpu_temp_level'] if self.current_values.get('cpu_temp_level') is not None else '',
                self.current_values['gpu_temp_level'] if self.current_values.get('gpu_temp_level') is not None else '',
                self.current_values['battery_level'] if self.current_values.get('battery_level') is not None else '',
                self.current_values['ram_level'] if self.current_values.get('ram_level') is not None else '',
                self.current_values['cpu_usage_level'] if self.current_values.get('cpu_usage_level') is not None else '',
                self.current_values['disk_temp_level'] if self.current_values.get('disk_temp_level') is not None else '',
                # Flags
                1 if any_warning else 0,
                1 if any_error else 0,
                1 if overheating_risk else 0,
                1 if battery_low else 0
            ]
            
            # Add to buffer
            self.data_buffer.append(row)
            
            # Write to disk if buffer is full
            if len(self.data_buffer) >= self.buffer_size:
                self._flush_buffer()

    def destroy_node(self):
        """Cleanup when destroying the node"""
        self._stop_logging()
        super().destroy_node()

def main():
    rclpy.init()
    node = SystemHealthRecorder()
    
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

if __name__ == '__main__':
    main()