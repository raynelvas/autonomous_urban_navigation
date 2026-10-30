#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool
from pathlib import Path
import cv2
import numpy as np
import pandas as pd
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DataReplayerManual(Node):
    def __init__(self):
        super().__init__('data_replayer_manual')
        
        # State
        self.current_log_path = None
        self.image_data = []
        self.current_image_index = 0
        self.lane_data = None  # DataFrame with data from lane_data.csv
        self.timestamps = None  #Array of timestamps from lane_data.csv
        
        # Autoplay status
        self.auto_play_enabled = False
        self.auto_play_timer = None
        
        # Publishers
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.image_pub = self.create_publisher(CompressedImage, '/image_raw/compressed', camera_qos)
        self.manual_pub = self.create_publisher(Bool, '/manual', 10)
        self.pub_change_left = self.create_publisher(Bool, '/change/left', 10)
        self.pub_change_right = self.create_publisher(Bool, '/change/right', 10)
        
        # Subscriber for the route
        self.create_subscription(String, '/replay_log_path', self.replay_path_cb, 10)
        
        # JPEG compression parameters
        self.compression_params = [cv2.IMWRITE_JPEG_QUALITY, 85]
        
        # Short timer just to refresh the OpenCV window and capture keyboard input.
        self.ui_timer = self.create_timer(0.01, self.update_ui)
        
        self.get_logger().info('Data Replayer Manual Initialized')
        self.get_logger().info('Controls:')
        self.get_logger().info('  [SPACE]    - Start/Stop automatic playback')
        self.get_logger().info('  [→] o [D]  - Next image')
        self.get_logger().info('  [←] o [A]  - Previous image')
        self.get_logger().info('  [E]  - Final image')
        self.get_logger().info('  [R]  - Initial image')
        self.get_logger().info('  [Q]        - Exit')

    def replay_path_cb(self, msg):
        """Callback when a new route arrives"""
        if msg.data == self.current_log_path:
            return
            
        # Stop automatic playback if it was active
        self.stop_auto_play()
        
        self.current_log_path = msg.data
        self.get_logger().info(f'======== New route received: {self.current_log_path} ========')
        
        # Upload lane images and data
        self._load_images()
        self._load_lane_data()
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
                    # Also load the perception CSV file if it exists.
                    self._load_perception_csv(base_path)
                    break
        
        self.image_data.sort()
        self.get_logger().info(f'--- loaded: {len(self.image_data)} images ---')

    def _load_perception_csv(self, base_path):
        """Load the perception_data.csv file to map image to timestamp"""
        perception_csv = base_path / "perception" / "perception_data.csv"
        if perception_csv.exists():
            try:
                self.perception_df = pd.read_csv(perception_csv)
                self.get_logger().info(f'Loaded perception_data.csv with{len(self.perception_df)} records')
            except Exception as e:
                self.get_logger().warning(f'Error loading perception_data.csv: {e}')
                self.perception_df = None
        else:
            self.perception_df = None
            self.get_logger().warning(f'perception_data.csv was not found in {base_path / "perception"}')

    def _load_lane_data(self):
        """Load the lane_data.csv file from robot_control."""
        self.lane_data = None
        lane_csv_path = Path(self.current_log_path) / "robot_control" / "lane_data.csv"
        
        if lane_csv_path.exists():
            try:
                self.lane_data = pd.read_csv(lane_csv_path)
                self.lane_data['timestamp'] = pd.to_numeric(self.lane_data['timestamp'])
                self.lane_data = self.lane_data.sort_values('timestamp')
                # Verify that the necessary columns exist.
                required_columns = ['timestamp', 'is_manual', 'change_left', 'change_right']
                missing_columns = [col for col in required_columns if col not in self.lane_data.columns]
                if missing_columns:
                    self.get_logger().warning(f'Missing columns in lane_data.csv: {missing_columns}')
                    self.lane_data = None
                else:
                    self.get_logger().info(f'Loaded lane_data.csv with {len(self.lane_data)} records')
            except Exception as e:
                self.get_logger().warning(f'Error loading lane_data.csv: {e}')
        else:
            self.get_logger().warning(f'lane_data.csv was not found in {lane_csv_path}')

    def _get_is_manual_for_image(self, image_filename):
        """
        Retrieves the is_manual value for an image based on its timestamp.
        Searches for the closest timestamp in lane_data.csv.
        """
        if self.lane_data is None or self.lane_data.empty:
            return False 
        
        # Get the image timestamp from perception_data.csv
        image_timestamp = self._get_timestamp_from_image(image_filename)
        
        if image_timestamp is None:
            self.get_logger().warning(f'No timestamp was found for {image_filename}')
            return False
        
        # Find the closest timestamp in lane_data
        idx = (self.lane_data['timestamp'] - image_timestamp).abs().idxmin()
        closest_timestamp = self.lane_data.loc[idx, 'timestamp']
        is_manual_value = self.lane_data.loc[idx, 'is_manual']
        
        # Convert to bool (1 -> True, 0 -> False)
        is_manual_bool = bool(is_manual_value)
        
        return is_manual_bool
    
    def _get_change_values_for_image(self, image_filename):
        """
        Obtains the change_left and change_right values ​​for an image
        based on its closest timestamp in lane_data.csv. 
        Searches within a range of ±5 indices around the closest match. 
        If any True value is found within that range, returns True for that side. 
        Returns: (change_left, change_right) as a tuple of booleans.
        """
        if self.lane_data is None or self.lane_data.empty:
            return False, False
        
        # Get the image timestamp
        image_timestamp = self._get_timestamp_from_image(image_filename)
        
        if image_timestamp is None:
            self.get_logger().warning(f'No se encontró timestamp para {image_filename}')
            return False, False
        
        # Find the index of the nearest timestamp.
        idx = (self.lane_data['timestamp'] - image_timestamp).abs().idxmin()
        position_idx = self.lane_data.index.get_loc(idx)
        
        # Define the search range
        start_idx = max(0, position_idx - 20)
        end_idx = min(len(self.lane_data) - 1, position_idx + 20)
        
        # Extract the range and search for True values ​​in a vectorized manner.
        range_data = self.lane_data.iloc[start_idx:end_idx + 1]
        
        # Check if there is at least one True in the range (handling NaN)
        change_left_found = False
        change_right_found = False

        # Explicitly convert to a Python bool
        if 'change_left' in range_data.columns:            
            left_values = range_data['change_left'].fillna(0).astype(bool)
            change_left_found = bool(left_values.any())  
        
        if 'change_right' in range_data.columns:
            right_values = range_data['change_right'].fillna(0).astype(bool)
            change_right_found = bool(right_values.any()) 
        
        return change_left_found, change_right_found

    def _get_timestamp_from_image(self, image_filename):
        """Gets the timestamp of an image from perception_data.csv."""
        if self.perception_df is None:
            return None
        
        try:
            # Search for the filename in the DataFrame.
            matching_rows = self.perception_df[self.perception_df['image_filename'] == image_filename]
            if not matching_rows.empty:
                return matching_rows.iloc[0]['timestamp']
        except Exception as e:
            self.get_logger().warning(f'Error searching for timestamp: {e}')
        
        return None

    def _display_and_publish(self):
        """It reads the current image, displays it, and publishes it along with the manual value."""
        if not self.image_data:
            return

        img_path = self.image_data[self.current_image_index]
        cv_image = cv2.imread(img_path)
        
        if cv_image is None:
            self.get_logger().warning(f'Error al leer: {img_path}')
            return

        # Extract only the filename from the full path.
        image_filename = Path(img_path).name
        
        # Get the value of is_manual for this image.
        is_manual = self._get_is_manual_for_image(image_filename)
        change_left, change_right = self._get_change_values_for_image(image_filename)
        
        # Resize for consistency
        cv_image = cv2.resize(cv_image, (512, 288), interpolation=cv2.INTER_AREA)

        # Show in a window with additional information
        display_img = cv_image.copy()
        auto_status = "AUTO" if self.auto_play_enabled else "MANUAL"
        info_text = f"{auto_status} | Img: {self.current_image_index + 1}/{len(self.image_data)} | Manual: {is_manual}"
        color = (0, 255, 0) if not is_manual else (0, 0, 255)  # Verde si no manual, rojo si manual
        cv2.putText(display_img, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Lane change indicators
        if change_left:
            cv2.putText(display_img, "CHANGE LEFT", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        if change_right:
            cv2.putText(display_img, "CHANGE RIGHT", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        # Autoplay indicator
        if self.auto_play_enabled:
            cv2.putText(display_img, "▶", (display_img.shape[1] - 30, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("Data Replayer - Manual Mode", display_img)

        # Publish image to ROS2
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.format = 'jpeg'
        
        success, encoded = cv2.imencode('.jpg', cv_image, self.compression_params)
        if success:
            msg.data = encoded.tobytes()
            self.image_pub.publish(msg)
            
            # Post manual value
            manual_msg = Bool()
            manual_msg.data = is_manual
            self.manual_pub.publish(manual_msg)

            # Publish change_left
            change_left_msg = Bool()
            change_left_msg.data = change_left
            self.pub_change_left.publish(change_left_msg)
            
            # Publish change_right
            change_right_msg = Bool()
            change_right_msg.data = change_right
            self.pub_change_right.publish(change_right_msg)

    def start_auto_play(self):
        """Start autoplay"""
        if self.auto_play_enabled:
            return
        
        if not self.image_data:
            self.get_logger().warning('No images loaded to display.')
            return
        
        self.auto_play_enabled = True
        self.auto_play_timer = self.create_timer(0.025, self.auto_play_step)

    def stop_auto_play(self):
        """Stops automatic playback"""
        if not self.auto_play_enabled:
            return
        
        self.auto_play_enabled = False
        if self.auto_play_timer is not None:
            self.auto_play_timer.cancel()
            self.auto_play_timer = None

    def auto_play_step(self):
        """Executes a step of the automatic playback."""
        if not self.auto_play_enabled or not self.image_data:
            return
        
        # Advance to the next frame
        next_index = self.current_image_index + 1
        
        # Check if we have reached the end.
        if next_index >= len(self.image_data):
            self.get_logger().info('Playback complete – Returning to the start')
            self.current_image_index = 0
            self.stop_auto_play()
        else:
            self.current_image_index = next_index
        
        self._display_and_publish()

    def update_ui(self):
        """Handles keyboard events"""
        key = cv2.waitKey(1) & 0xFF
        
        if not self.image_data:
            return

        # SPACE key: Start/Stop automatic playback
        if key == ord(' '):
            if self.auto_play_enabled:
                self.stop_auto_play()
            else:
                self.start_auto_play()
            
        # Right Arrow Key (or 'd')
        elif key == 83 or key == ord('d'):
            # If autoplay is active, stop it upon manual interaction.
            if self.auto_play_enabled:
                self.stop_auto_play()
            self.current_image_index = (self.current_image_index + 1) % len(self.image_data)
            self._display_and_publish()
            
        # Left Arrow Key (or 'a')
        elif key == 81 or key == ord('a'):
            # If autoplay is active, stop it upon manual interaction.
            if self.auto_play_enabled:
                self.stop_auto_play()
            self.current_image_index = (self.current_image_index - 1) % len(self.image_data)
            self._display_and_publish()
            
        # R Key: Restart from the beginning
        elif key == ord('r') or key == ord('R'):
            if self.auto_play_enabled:
                self.stop_auto_play()
            self.current_image_index = 0
            self._display_and_publish()
            
        # E key: Advance to the last frame
        elif key == ord('e') or key == ord('E'):
            if self.auto_play_enabled:
                self.stop_auto_play()
            self.current_image_index = len(self.image_data) - 1
            self._display_and_publish()
            
        # Go out with 'q'
        elif key == ord('q'):
            self.get_logger().info('Cerrando...')
            self.stop_auto_play()
            cv2.destroyAllWindows()
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