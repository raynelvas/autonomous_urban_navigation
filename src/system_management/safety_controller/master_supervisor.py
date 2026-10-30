#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus
import threading
from collections import defaultdict
from std_msgs.msg import Bool

class StatusMonitor(Node):
    def __init__(self):
        super().__init__('master_supervisor')
        
        # Parameters
        self.declare_parameter('status_topic_prefix', '/status/')
        self.declare_parameter('global_status_topic', '/global_status')
        self.declare_parameter('discovery_interval', 2.0)  # seconds
        self.declare_parameter('warning_blink_interval', 1.0)  # seconds for blinking
        
        self.status_topic_prefix = self.get_parameter('status_topic_prefix').value
        self.global_status_topic = self.get_parameter('global_status_topic').value
        self.discovery_interval = self.get_parameter('discovery_interval').value
        self.warning_blink_interval = self.get_parameter('warning_blink_interval').value
        
        # Publisher for the global state
        self.global_status_publisher = self.create_publisher(
            DiagnosticStatus, 
            self.global_status_topic, 
            10
        )

        self.light_safety_pub = self.create_publisher(Bool, '/light/safety', 10)
        
        # Dictionaries for storing states and subscribers
        self.status_data = {}  # {topic_name: DiagnosticStatus}
        self.status_subscriptions = {}  # {topic_name: subscription}

        # Flicker control variables
        self.current_global_state = DiagnosticStatus.STALE  # Estado inicial
        self.blink_state = False  # Estado actual del parpadeo (True=ON, False=OFF)
        self.blink_timer = None  # Timer para el parpadeo
        
        self.lock = threading.Lock()
        
        # Timer for discovering new topics
        self.discovery_timer = self.create_timer(self.discovery_interval, self.discover_status_topics)
        
        # Discover initial topics
        self.discover_status_topics()
    
    def discover_status_topics(self):
        """Discover and subscribe to new status topics"""
        try:
            topic_names_and_types = self.get_topic_names_and_types()
            status_topics = []
            
            for topic_name, topic_types in topic_names_and_types:
                # Search for topics that start with the prefix and are of the correct type.
                if (topic_name.startswith(self.status_topic_prefix) and 
                    'diagnostic_msgs/msg/DiagnosticStatus' in topic_types):
                    status_topics.append(topic_name)
            
            with self.lock:
                # Subscribe to new topics
                for topic in status_topics:
                    if topic not in self.status_subscriptions:
                        self.create_subscription_for_topic(topic)
                
                # Remove subscriptions to topics that no longer exist
                topics_to_remove = []
                for existing_topic in self.status_subscriptions.keys():
                    if existing_topic not in status_topics:
                        topics_to_remove.append(existing_topic)
                
                for topic in topics_to_remove:
                    self.remove_subscription_for_topic(topic)
                        
        except Exception as e:
            pass
    
    def create_subscription_for_topic(self, topic_name):
        """Create a subscription for a specific topic."""
        try:
            subscription = self.create_subscription(
                DiagnosticStatus,
                topic_name,
                lambda msg, tn=topic_name: self.status_callback(msg, tn),
                10
            )
            self.status_subscriptions[topic_name] = subscription
            self.status_data[topic_name] = None  # Unknown initial state
            
        except Exception as e:
            pass
    
    def remove_subscription_for_topic(self, topic_name):
        """Deletes a subscription for a specific topic."""
        if topic_name in self.status_subscriptions:
            del self.status_subscriptions[topic_name]
            if topic_name in self.status_data:
                del self.status_data[topic_name]
    
    def status_callback(self, msg, topic_name):
        """Callback to process status messages"""
        with self.lock:
            self.status_data[topic_name] = msg
            self.evaluate_global_status()
    
    def start_blinking(self):
        """Starts the blinking timer for the WARNING state."""
        if self.blink_timer is None:
            self.blink_timer = self.create_timer(
                self.warning_blink_interval, 
                self.blink_callback
            )
            self.blink_state = False  
    
    def stop_blinking(self):
        """Stops the blinking timer."""
        if self.blink_timer is not None:
            self.blink_timer.destroy()
            self.blink_timer = None
    
    def blink_callback(self):
        """Callback to control flickering in the WARNING state"""
        if self.current_global_state == DiagnosticStatus.WARN:
            self.blink_state = not self.blink_state
            safety_relay = Bool()
            safety_relay.data = self.blink_state
            self.light_safety_pub.publish(safety_relay)

    def evaluate_global_status(self):
        """Evaluates the overall state based on all individual states"""
        if not self.status_data:
            return
        
        status_counts = defaultdict(int)
        components_with_data = 0
        
        for topic, status_msg in self.status_data.items():
            if status_msg is not None:
                components_with_data += 1
                status_level = status_msg.level
                status_counts[status_level] += 1
        
        # Do not publish if there is insufficient data.
        if components_with_data == 0:
            return
        
        # Determine global state
        global_status = DiagnosticStatus()
        global_status.name = "Global System State"
        
        safety_relay = Bool()
        previous_state = self.current_global_state

        # Simple priority logic
        if status_counts.get(DiagnosticStatus.ERROR, 0) > 0:
            safety_relay.data = False
            global_status.level = DiagnosticStatus.ERROR
            global_status.message = "ERROR"
            self.current_global_state = DiagnosticStatus.ERROR

        elif status_counts.get(DiagnosticStatus.WARN, 0) > 0:
            global_status.level = DiagnosticStatus.WARN
            global_status.message = "ADVERTENCIA"
            self.current_global_state = DiagnosticStatus.WARN

        elif status_counts.get(DiagnosticStatus.OK, 0) == components_with_data:
            safety_relay.data = True
            global_status.level = DiagnosticStatus.OK
            global_status.message = "OK"
            self.current_global_state = DiagnosticStatus.OK

        else:
            global_status.level = DiagnosticStatus.WARN
            global_status.message = "UNKNOWN"
            self.current_global_state = DiagnosticStatus.WARN
        
        # Post global status
        self.global_status_publisher.publish(global_status)
            
        # Control blinking based on state changes
        if previous_state != self.current_global_state:
            if self.current_global_state == DiagnosticStatus.WARN:
                self.start_blinking()
            else:
                self.stop_blinking()
                self.light_safety_pub.publish(safety_relay)
        elif self.current_global_state != DiagnosticStatus.WARN:
            self.light_safety_pub.publish(safety_relay)

    def destroy_node(self):
        self.stop_blinking()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    status_monitor = StatusMonitor()
    
    try:
        rclpy.spin(status_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            status_monitor.destroy_node()
            rclpy.shutdown()
        except:
            pass  

if __name__ == '__main__':
    main()