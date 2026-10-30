#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import CompressedImage
import time

class CameraSupervisor(Node):
    def __init__(self):
        super().__init__('camera_supervisor')

        # Camera state publisher
        self.publisher = self.create_publisher(DiagnosticStatus, '/status/camera', 10)

        # Watchdog timer
        self.timer = self.create_timer(2.0, self.check_camera)

        # Cache to detect state changes
        self.last_camera_state = {
            'online': False, 
            'topics_found': 0, 
            'level': 2
        }

        # Last activity timestamps
        self.last_activity_time = None

        # Received message counter
        self.message_counter = 0

        # Inactivity timeout (seconds)
        self.ACTIVITY_TIMEOUT = 8.0  

        # Expected camera topics
        self.expected_topics = [
            '/image_raw/compressed'
        ]

        # Critical topics for monitoring activity
        self.critical_topics = [
            '/image_raw/compressed'
        ]

        # SUBSCRIPTIONS
        best_effort_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.subscription_camera = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.camera_listener,
            best_effort_qos 
        )

    def camera_listener(self, msg):
        """Callback that receives compressed images and logs activity"""
        current_time = time.time()
        self.last_activity_time = current_time
        self.message_counter += 1
        
    def check_activity_timeout(self):
        """Check if the camera has been inactive for longer than the timeout period."""
        current_time = time.time()
        last_activity = self.last_activity_time
        
        if last_activity is None:
            return True
        
        time_since_activity = current_time - last_activity
        return time_since_activity > self.ACTIVITY_TIMEOUT

    def get_topic_names(self):
        """Retrieves a list of active topics with retries."""
        for attempt in range(3):
            try:
                topic_names_and_types = self.get_topic_names_and_types()
                return [name for name, types in topic_names_and_types]
            except Exception as e:
                if attempt == 2:
                    self.get_logger().error(f"Error retrieving topics: {e}")
                time.sleep(0.1)
        return []

    def check_camera(self):
        """Main function that checks the camera status"""
        try:
            # Get current list of topics
            topic_list = self.get_topic_names()

            msg = DiagnosticStatus()
            msg.name = "Cámara"
            msg.hardware_id = "RGBD_Camera"

            # Verify camera topics
            found_topics = [t for t in self.expected_topics if t in topic_list]
            missing_topics = [t for t in self.expected_topics if t not in topic_list]
            found_topics_count = len(found_topics)

            # Check activity timeout
            activity_timed_out = self.check_activity_timeout()
            
            # Determine current status
            if found_topics_count == 0:
                # No camera topics were detected.
                msg.level = DiagnosticStatus.ERROR  # 2 = ERROR
                msg.message = "DESCONECTADA"
                camera_online = False

                self.last_activity_time = None
                
            elif activity_timed_out and found_topics_count > 0:
                # ZOMBIE CHAMBER: Topics present but inactive.
                msg.level = DiagnosticStatus.ERROR  # 2 = ERROR  
                msg.message = "INACTIVA (TIMEOUT)"
                camera_online = False
                
            else:
                # Camera active and communicating
                camera_online = True

                # Assess level based on identified topics.
                if len(missing_topics) == 0:
                    msg.level = DiagnosticStatus.OK
                    msg.message = f"Conectada- {found_topics_count}/{len(self.expected_topics)} tópicos activos"
                elif len(missing_topics) <= 2:  
                    msg.level = DiagnosticStatus.WARN
                    msg.message = f"CONECTADA - Faltan {len(missing_topics)} tópicos"
                else:
                    msg.level = DiagnosticStatus.WARN
                    msg.message = f"CONECTADA - Faltan {len(missing_topics)} tópicos"

            # Additional information
            current_time = time.time()
            last_activity = self.last_activity_time
            time_since_activity = "Nunca" if last_activity is None else f"{current_time - last_activity:.1f}s"
            
            msg.values = [
                KeyValue(key="expected_topics", value=str(len(self.expected_topics))),
                KeyValue(key="found_topics", value=str(found_topics_count)),
                KeyValue(key="missing_topics_count", value=str(len(missing_topics))),
                KeyValue(key="missing_topics_list", value=", ".join(missing_topics) if missing_topics else "Ninguno"),
                KeyValue(key="status", value="ONLINE" if camera_online else "OFFLINE"),
                KeyValue(key="timestamp", value=str(current_time)),
                KeyValue(key="last_activity", value=time_since_activity),
                KeyValue(key="activity_timeout", value=str(self.ACTIVITY_TIMEOUT)),
                KeyValue(key="activity_timed_out", value=str(activity_timed_out)),
                KeyValue(key="message_count", value=str(self.message_counter)),
                KeyValue(key="critical_topics", value=", ".join(self.critical_topics)),
            ]

            # Log control to avoid spam
            current_state = {
                'online': camera_online,
                'topics_found': found_topics_count,
                'level': msg.level,
                'activity_timed_out': activity_timed_out
            }

            # Only log if there are significant changes
            if (self.last_camera_state['online'] != current_state['online'] or
                self.last_camera_state['topics_found'] != current_state['topics_found'] or
                self.last_camera_state['level'] != current_state['level'] or
                self.last_camera_state['activity_timed_out'] != current_state['activity_timed_out']):
                
                self.last_camera_state = current_state

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error in check_camera: {e}")
            pass

    def destroy_node(self):
        try:
            if hasattr(self, 'subscription_camera'):
                self.destroy_subscription(self.subscription_camera)
        except Exception as e:
            self.get_logger().warning(f"Error en cleanup: {e}")
            pass
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraSupervisor()
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