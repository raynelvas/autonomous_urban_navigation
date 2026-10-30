#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
import csv
import time
import cv2
import numpy as np
from datetime import datetime

class PerceptionRecorder(Node):
    def __init__(self):
        super().__init__('perception_recorder')
        
        # Sampling parameter
        self.declare_parameter('save_every_n', 10)
        
        # State
        self.is_recording = False
        self.current_log_path = None
        self.frame_count = 0
        self.saved_frames = 0
        
        # Bridge
        self.bridge = CvBridge()
        
        # --- SUBSCRIPTIONS ---
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        self.create_subscription(Image, '/image_raw', self.image_callback, qos)
        
        # Manager's signs
        self.create_subscription(Bool, '/logging_enabled', self.recording_enabled_cb, 10)
        self.create_subscription(String, '/current_log_path', self.log_path_cb, 10)
        
        # Files
        self.csv_file = None
        self.csv_writer = None
        self.current_session_dir = None

    def recording_enabled_cb(self, msg):
        if msg.data != self.is_recording:
            self.is_recording = msg.data
            if self.is_recording:
                if self.current_log_path:
                    self._start_recording()
            else:
                self._stop_recording()

    def log_path_cb(self, msg):
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            if self.is_recording and not self.current_session_dir:
                self._start_recording()

    def image_callback(self, msg):
        """DIRECT PROCESSING: When an image arrives, it saves it (if recording)."""
        if not self.is_recording:
            return
        
        try:
            # Increment received frames counter
            self.frame_count += 1
            
            # Verify sampling
            save_every_n = self.get_parameter('save_every_n').value
            if self.frame_count % save_every_n != 0:
                return

            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if image is None:
                return
            
            # Save image
            self.saved_frames += 1
            frame_id = f"frame_{self.saved_frames:06d}"
            timestamp = time.time()
            
            # Save JPEG
            img_filename = f"{frame_id}.jpg"
            img_path = self.current_session_dir / "images" / img_filename
            cv2.imwrite(str(img_path), image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            
            # Save CSV
            if self.csv_writer:
                row = [timestamp, frame_id, str(img_filename)]
                self.csv_writer.writerow(row)
                self.csv_file.flush()
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def _start_recording(self):
        if not self.current_log_path:
            return
            
        try:
            base_dir = Path(self.current_log_path)
            self.current_session_dir = base_dir / "perception"
            self.current_session_dir.mkdir(parents=True, exist_ok=True)
            
            # Create image folder
            images_dir = self.current_session_dir / "images"
            images_dir.mkdir(exist_ok=True)
            
            # CSV
            csv_path = self.current_session_dir / "perception_data.csv"
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'frame_id', 'image_filename'])
            self.csv_file.flush()
            
            # Reset counters
            self.frame_count = 0
            self.saved_frames = 0

        except Exception as e:
            self.get_logger().error(f'Error starting up: {e}')

    def _stop_recording(self):
        if self.saved_frames > 0:
            # Simple summary
            try:
                summary_path = self.current_session_dir / "summary.txt"
                with open(summary_path, 'w') as f:
                    f.write(f"Total imágenes: {self.saved_frames}\n")
                    f.write(f"Fecha: {datetime.now()}\n")
            except:
                pass
        
        # Cerrar CSV
        if self.csv_file:
            self.csv_file.close()
        
        # Resetear
        self.csv_file = None
        self.csv_writer = None
        self.current_session_dir = None
        self.frame_count = 0
        self.saved_frames = 0

    def destroy_node(self):
        self._stop_recording()
        super().destroy_node()

def main():
    rclpy.init()
    node = PerceptionRecorder()
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