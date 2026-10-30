import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')

        # ============ Declare parameters in the launch ============
        self.declare_parameter('save_directory', '~/Documents/generar_video/ruta18/lane_plus_segmetnation')

        self.save_path = self.get_parameter('save_directory').get_parameter_value().string_value
        # =============================================

        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            
        self.bridge = CvBridge()
        self.counter = 0
        
        # Topic subscriber
        self.subscription = self.create_subscription(
            Image,
            '/debug/camera_front',
            self.listener_callback,
            10)
        
        self.get_logger().info(f'Node started. Saving images to: {self.save_path}')

    def listener_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Create unique filename
            filename = os.path.join(self.save_path, f'frame_{self.counter:05d}.jpg')
            
            # Save image
            cv2.imwrite(filename, cv_image)
            self.counter += 1
            
            if self.counter % 10 == 0:
                self.get_logger().info(f'Saved {self.counter} images...')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
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