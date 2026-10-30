#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from std_msgs.msg import String
from pathlib import Path
import cv2
import numpy as np
from cv_bridge import CvBridge 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DataReplayerManual(Node):
    def __init__(self):
        super().__init__('data_replayer_manual')
        
        self.bridge = CvBridge()
        self.current_log_path = None
        self.image_paths = []
        self.current_image_index = 0
        
        # Publisher
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.image_pub = self.create_publisher(Image, '/image_raw', camera_qos)
        
        # Subscriber for the route
        self.create_subscription(String, '/replay_log_path', self.replay_path_cb, 10)
        
        # Short timer just to refresh the OpenCV window and capture keyboard input.
        self.ui_timer = self.create_timer(0.01, self.update_ui)

        self.get_logger().info(' Initialized Player Manual')
        self.get_logger().info('Controls: [Right Arrow] Next, [Left Arrow] Previous')
 
    def replay_path_cb(self, msg):
        """Callback when a new route arrives"""
        path_str = msg.data.strip()
        if path_str == self.current_log_path:
            return
            
        self.current_log_path = path_str
        self.get_logger().info(f'======== New route received: {self.current_log_path} ========')

        # Load and restart
        self._load_images()
        self.current_image_index = 0
        
        if self.image_paths:
            self._display_and_publish()

    def _load_images(self):
        self.image_paths = []
        base_path = Path(self.current_log_path)
        
        # We search through the folders.
        search_dirs = [base_path / "perception/images", base_path / "perception_frontal/images"]
        
        for directory in search_dirs:
            if directory.exists():
                for ext in ['.jpg', '.jpeg', '.png']:
                    self.image_paths.extend(list(directory.glob(f'*{ext}')))
                if self.image_paths: break
        
        self.image_paths.sort()
        self.get_logger().info(f'--- loaded: {len(self.image_data)} images ---')

    def _display_and_publish(self):
        """Reads the current image, displays it, and publishes it."""
        if not self.image_paths:
            return

        img_path = str(self.image_paths[self.current_image_index])
        cv_image = cv2.imread(img_path)
        
        if cv_image is None:
            self.get_logger().warning(f'⚠️ Error al leer: {img_path}')
            return

        # 1. Show in window
        display_img = cv_image.copy()
        info_text = f"Frame: {self.current_image_index + 1}/{len(self.image_paths)}"
        cv2.putText(display_img, info_text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("Jetson Replayer - Manual", display_img)

        # 2. Publish to ROS2
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_link'
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')

    def update_ui(self):
        """Handles keyboard events"""
        key = cv2.waitKey(1) & 0xFF
        if not self.image_paths: return

        # Right Arrow Key (or 'd')
        if key in [ord('d'), 83]: 
            self.current_image_index = (self.current_image_index + 1) % len(self.image_paths)
            self._display_and_publish()

        # Left Arrow Key (or 'a')
        elif key in [ord('a'), 81]: 
            self.current_image_index = (self.current_image_index - 1) % len(self.image_paths)
            self._display_and_publish()

        # Go out with 'q'
        elif key == ord('q'):
            self.get_logger().info('Closing..')
            rclpy.shutdown()

def main():
    rclpy.init()
    node = DataReplayerManual()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass 

if __name__ == '__main__':
    main()