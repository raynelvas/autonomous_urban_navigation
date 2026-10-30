#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from pathlib import Path
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DataReplayerManual(Node):
    def __init__(self):
        super().__init__('data_replayer_manual')
        
        self.current_log_path = None
        self.image_data = []
        self.current_image_index = 0
        
        # Publisher
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.image_pub = self.create_publisher(CompressedImage, '/image_raw/compressed', camera_qos)
        
        # Subscriber for the route
        self.create_subscription(String, '/replay_log_path', self.replay_path_cb, 10)
        
        # JPEG compression parameters
        self.compression_params = [cv2.IMWRITE_JPEG_QUALITY, 85]
        
        # Short timer just to refresh the OpenCV window and capture keyboard input.
        self.ui_timer = self.create_timer(0.01, self.update_ui)
        
        self.get_logger().info(' Initialized Player Manual')
        self.get_logger().info('Controls: [Right Arrow] Next, [Left Arrow] Previous')

    def replay_path_cb(self, msg):
        """Callback when a new route arrives"""
        if msg.data == self.current_log_path:
            return
            
        self.current_log_path = msg.data
        self.get_logger().info(f'======== New route received: {self.current_log_path} ========')
        
        # Load and restart
        self._load_images()
        self.current_image_index = 0
        
        if self.image_data:
            self._display_and_publish()

    def _load_images(self):
        self.image_data = []
        base_path = Path(self.current_log_path)
        
        # We search through the folders.
        possible_paths = [
            base_path / "perception" / "images",
            base_path / "perception_frontal" / "images"
        ]
        
        for image_dir in possible_paths:
            if image_dir.exists():
                extensions = ['.jpg', '.jpeg', '.png']
                for ext in extensions:
                    for img_path in sorted(image_dir.glob(f'*{ext}')):
                        self.image_data.append(str(img_path))
                if self.image_data:
                    break
        
        self.image_data.sort()
        self.get_logger().info(f'--- loaded: {len(self.image_data)} images ---')

    def _display_and_publish(self):
        """Reads the current image, displays it, and publishes it."""
        if not self.image_data:
            return

        img_path = self.image_data[self.current_image_index]
        cv_image = cv2.imread(img_path)
        
        if cv_image is None:
            self.get_logger().warning(f'⚠️ Error al leer: {img_path}')
            return

        # Resize for consistency
        cv_image = cv2.resize(cv_image, (512, 288), interpolation=cv2.INTER_AREA)

        # 1. Show in window
        display_img = cv_image.copy()
        info_text = f"Img: {self.current_image_index + 1}/{len(self.image_data)}"
        cv2.putText(display_img, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("Data Replayer - Manual Mode", display_img)

        # 2. Publish to ROS2
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.format = 'jpeg'
        
        success, encoded = cv2.imencode('.jpg', cv_image, self.compression_params)
        if success:
            msg.data = encoded.tobytes()
            self.image_pub.publish(msg)

    def update_ui(self):
        """Handles keyboard events"""
        key = cv2.waitKey(1) & 0xFF
        
        if not self.image_data:
            return

        # Right Arrow Key (or 'd')
        if key == 83 or key == ord('d'): 
            self.current_image_index = (self.current_image_index + 1) % len(self.image_data)
            self._display_and_publish()
            
        # Left Arrow Key (or 'a')
        elif key == 81 or key == ord('a'): 
            self.current_image_index = (self.current_image_index - 1) % len(self.image_data)
            self._display_and_publish()
            
        # Go out with 'q'
        elif key == ord('q'):
            self.get_logger().info('Closing...')
            rclpy.shutdown()

def main():
    rclpy.init()
    node = DataReplayerManual()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt):
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