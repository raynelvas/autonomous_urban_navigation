#!/usr/bin/env python3
"""
Manager Node for Autonomous Robot Logging System
This node manages folder creation and sends start/stop signals
to all logging nodes based on specific conditions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import os
import datetime
import time
from pathlib import Path

class LoggingManager(Node):
    def __init__(self):
        super().__init__('logging_manager')
        
        # Configurable parameters
        self.declare_parameter('base_log_dir', '~/autonomous_navigation/src/saves/data_logs')
        self.declare_parameter('log_prefix', 'ruta')
        self.declare_parameter('enable_auto_naming', True)
        self.declare_parameter('manual_route_name', '')
        
        # Get parameters
        self.base_log_dir = Path(self.get_parameter('base_log_dir').value).expanduser()
        self.log_prefix = self.get_parameter('log_prefix').value
        self.enable_auto_naming = self.get_parameter('enable_auto_naming').value
        self.manual_route_name = self.get_parameter('manual_route_name').value
        
        # Internal state
        self.is_logging = False
        self.current_log_dir = None
        self.lid_closed = False
        self.record = False
        
        # Variables to detect transitions
        self.previous_lid_closed = False  # Previous state of lid_closed
        self.lid_closed_transition = False  # Indicates if there was F->T transition

        self.previous_record = False  # Previous record status
        self.record_transition = False  # Indicates if there was F->T transition in record
        
        # Counter for unique names
        self.route_counter = self._get_next_route_number()
        
        # Publishers
        self.logging_signal_pub = self.create_publisher(Bool, '/logging_enabled', 10)
        self.logging_path_pub = self.create_publisher(String, '/current_log_path', 10)
        
        # Subscribers
        self.create_subscription(Bool, '/lid_closed', self.lid_closed_callback, 10)
        self.create_subscription(Bool, '/record', self.record_callback, 10)
        
        # Timer for periodic checkup
        self.timer = self.create_timer(0.1, self.check_conditions)  # 10 Hz
        
    def _get_next_route_number(self):
        """Gets next path number based on existing folders"""
        if not self.base_log_dir.exists():
            return 1
            
        existing_folders = [d for d in self.base_log_dir.iterdir() if d.is_dir()]
        max_num = 0
        
        for folder in existing_folders:
            folder_name = folder.name
            # Look for patterns like "ruta1", "ruta2_", etc.
            if folder_name.startswith(self.log_prefix):
                try:
                    # Extract the number after the prefix
                    remaining = folder_name[len(self.log_prefix):]
                    # Find the first number
                    num_str = ''
                    for char in remaining:
                        if char.isdigit():
                            num_str += char
                        elif num_str:  # If we already start numbers and find non-digit
                            break
                    if num_str:
                        num = int(num_str)
                        max_num = max(max_num, num)
                except (ValueError, IndexError):
                    continue
        
        return max_num + 1
    
    def _generate_route_name(self):
        """Generate a unique name for the path folder"""
        if not self.enable_auto_naming and self.manual_route_name:
            base_name = self.manual_route_name
        else:
            # Simply return the next available number
            while True:
                candidate_name = f"{self.log_prefix}{self.route_counter}"
                full_path = self.base_log_dir / candidate_name
                
                if not full_path.exists():
                    return candidate_name
                else:
                    self.route_counter += 1
    
    def _create_log_directory(self, route_name):
        """Create the log folder with subfolders if necessary"""
        log_dir = self.base_log_dir / route_name
        
        try:
            log_dir.mkdir(parents=True, exist_ok=False)
            
            # Create metadata file
            metadata_file = log_dir / "metadata.txt"
            with open(metadata_file, 'w') as f:
                f.write(f"Path: {route_name}\n")
                f.write(f"Date: {datetime.datetime.now().isoformat()}\n")
                f.write(f"Manager Node: {self.get_name()}\n")
                f.write("Initial state:\n")
                f.write(f"  lid_closed: {self.lid_closed}\n")
            
            return log_dir
            
        except FileExistsError:
            self.get_logger().error(f'The directory {log_dir} already exists')
            return None
        except Exception as e:
            self.get_logger().error(f'Error creating directory: {e}')
            return None
    
    def lid_closed_callback(self, msg):
        """Callback for topic /lid_closed"""
        new_value = msg.data
        
        # Detect transition from False to True
        if not self.previous_lid_closed and new_value:
            self.lid_closed_transition = True
        elif self.previous_lid_closed and not new_value:
            pass
        
        # Update statuses
        self.previous_lid_closed = self.lid_closed
        self.lid_closed = new_value
    
    def record_callback(self, msg):
        """Callback for the /record topic"""
        new_value = msg.data
        
        # Detect transition from False to True
        if not self.previous_record and new_value:
            self.record_transition = True
        elif self.previous_record and not new_value:
            pass
        
        # Update statuses
        self.previous_record = self.record
        self.record = new_value        
        
    def start_logging(self):
        """Starts the logging process"""
        if self.is_logging:
            return False
        
        try:
            # Generate name and create folder
            route_name = self._generate_route_name()
            self.current_log_dir = self._create_log_directory(route_name)
            
            if not self.current_log_dir:
                return False
            
            # Publish current route to all nodes
            path_msg = String()
            path_msg.data = str(self.current_log_dir)
            self.logging_path_pub.publish(path_msg)
            
            # Wait a moment for the nodes to receive the route.
            time.sleep(0.1)
            
            # Send start signal
            self._publish_logging_signal(True)
            self.is_logging = True
            self.route_counter += 1
            
            # Reset transition flag
            self.lid_closed_transition = False
            
            return True
            
        except Exception as e:
            return False
    
    def stop_logging(self):
        """Stops the logging process"""
        if not self.is_logging:
            return False
        
        try:
            # Send stop signal
            self._publish_logging_signal(False)
            self.is_logging = False
            
            # Update metadata file with final information
            if self.current_log_dir:
                metadata_file = self.current_log_dir / "metadata.txt"
                with open(metadata_file, 'a') as f:
                    f.write(f"\nLogging finalizado: {datetime.datetime.now().isoformat()}\n")
                    f.write("Motivo: condicion_falsa\n")
                    f.write(f"Estado final:\n")
                    f.write(f"  lid_closed: {self.lid_closed}\n")
            
            self.current_log_dir = None
            return True
            
        except Exception as e:
            return False
    
    def _publish_logging_signal(self, enable):
        """Publishes the logging signal to all nodes"""
        msg = Bool()
        msg.data = enable
        self.logging_signal_pub.publish(msg)
    
    def check_conditions(self):
        """Periodically checks conditions to start/stop logging."""
        
        # CONDITION TO START:
        # There was a transition from False to True in lid_closed or record
        # Must not already be logging
        start_condition = (
            (self.lid_closed_transition and self.lid_closed) # lid F->T transition detected lid_closed
             or (self.record_transition and self.record) # F->T transition detected in register
             and not self.is_logging             # It shouldn't be logging anymore.         
        )
        
        # CONDITIONS FOR STOPPING:
        stop_condition = (
            (not self.lid_closed and not self.record) 
        )
        
        # State transition logic
        if start_condition:
            self.start_logging()
            
        elif self.is_logging and stop_condition:
            reason = []
            if not self.lid_closed:
                reason.append('lid_closed=False')
            
            self.stop_logging()
    
    def destroy_node(self):
        """Cleanup upon node destruction"""
        if self.is_logging:
            self.stop_logging()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        logging_manager = LoggingManager()
        rclpy.spin(logging_manager)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if 'logging_manager' in locals():
                logging_manager.destroy_node()
            rclpy.shutdown()
        except:
            pass 

if __name__ == '__main__':
    main()