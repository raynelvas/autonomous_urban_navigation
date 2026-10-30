#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
import os
import threading
import numpy as np

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')

        # ============ Declare parameters in the launch ============
        self.declare_parameter('save_directory', '/home/raynel/Documents')

        self.save_directory = self.get_parameter('save_directory').get_parameter_value().string_value
        # =============================================
        
        os.makedirs(self.save_directory, exist_ok=True)

        self.bridge = CvBridge()
        
        # Variables to store the last image
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        # QoS Best Effort for the compressed image
        best_effort_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.image_sub = self.create_subscription(
            CompressedImage,  
            '/image_raw/compressed',  
            self.image_callback,
            best_effort_qos  
        )
        
        self.capture_sub = self.create_subscription(
            Bool,
            '/capture',
            self.capture_callback,
            10
        )
        
        # Image counter
        self.image_count = self.get_next_image_number()
    
    def get_next_image_number(self):
        """Get the next available number to name images"""
        try:
            # Search for files that match the pattern image_XXXXX.jpg
            existing_files = [f for f in os.listdir(self.save_directory) 
                            if f.startswith('image_') and f.endswith('.jpg')]
            
            if not existing_files:
                return 1
            
            numbers = []
            for filename in existing_files:
                try:
                    num_str = filename[6:-4]  # 'image_00001.jpg' -> '00001'
                    numbers.append(int(num_str))
                except ValueError:
                    continue
            
            if numbers:
                return max(numbers) + 1
            else:
                return 1
                
        except Exception as e:
            return 1
    
    def image_callback(self, msg):
        """Callback to receive images from the camera"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            with self.image_lock:
                self.latest_image = image
                
        except Exception as e:
            pass
    
    def capture_callback(self, msg):
        """Callback to capture image when Bool message is received"""
        if msg.data:
            self.capture_image()
    
    def capture_image(self):
        """Captures and saves the current image"""
        if self.latest_image is None:
            return
        
        try:
            with self.image_lock:
                filename = f"image_{self.image_count:05d}.jpg"
                filepath = os.path.join(self.save_directory, filename)
                
                cv2.imwrite(filepath, self.latest_image)

                self.image_count += 1
                
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    node = ImageCaptureNode()
    
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