#!/usr/bin/env python3
"""
DataReplayer WITH SIMPLIFIED CALIBRATION - Replays data with live time
========================================================================
- Replays odometry, IMU, and prerecorded images 
- Works with any combination of data (images only, sensors only, or both)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, CameraInfo, CompressedImage
from std_msgs.msg import String, Bool
import csv
from pathlib import Path
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import pandas as pd

class DataReplayer(Node):
    def __init__(self):
        super().__init__('data_replayer')
        
        # Configuration parameters
        self.declare_parameter('replay_speed', 1.0)
        self.replay_speed = self.get_parameter('replay_speed').value
        
        # Replay state
        self.current_log_path = None
        self.start_time = None
        self.real_start_time = None
        self.has_sensors = False
        self.has_images = False
        
        # For calibration mode
        self.first_image_path = None
        self.first_image_timestamp = None
        
        # Data buffers
        self.sensor_data = []
        self.image_data = []  # Front camera only
        self.lane_data = None  # DataFrame with data from lane_data.csv
        self.perception_df = None  # DataFrame with data from perception_data.csv
        
        # Current indices
        self.current_data_index = 0
        self.current_image_index = 0
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom/unfiltered', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/unfiltered', 10)
        self.manual_pub = self.create_publisher(Bool, '/manual', 10)
        self.pub_change_left = self.create_publisher(Bool, '/change/left', 10)
        self.pub_change_right = self.create_publisher(Bool, '/change/right', 10)
        
        # Publisher for front camera (best effort)
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.image_pub = self.create_publisher(CompressedImage, '/image_raw/compressed', camera_qos)
        
        # Timers
        self.replay_timer = self.create_timer(0.01, self.replay_callback)  # 100 Hz
        self.replay_timer.cancel()
        
        # Single subscriber for the path
        self.create_subscription(String, '/replay_log_path', self.replay_path_cb, 10)
        
        # JPEG compression parameters
        self.compression_params = [cv2.IMWRITE_JPEG_QUALITY, 85]
        
        self.get_logger().info('DataReplayer WITH SIMPLIFIED CALIBRATION initialized')

    def replay_path_cb(self, msg):
        """Callback to receive the path and start automatically"""
        if msg.data == self.current_log_path:
            return
            
        self.current_log_path = msg.data
        self.get_logger().info(f'Path received: {self.current_log_path}')
        
        # Load data and start replay
        self._load_all_data()
        self._start_reproduction()

    def _start_reproduction(self):
        """Starts replay according to mode and available data"""
        # Configure base time
        self._setup_time_reference()
        
        # Reset indices
        self.current_data_index = 0
        self.current_image_index = 0
        
        self.replay_timer.reset()


    def _setup_time_reference(self):
        """Sets the time reference based on available data"""
        # Priority: sensors > images
        if self.has_sensors and self.sensor_data:
            self.start_time = self.sensor_data[0]['stamp']
            self.get_logger().info(f'Time reference: first sensor at t={self.start_time:.3f}s')
        elif self.has_images and self.image_data:
            self.start_time = self.image_data[0]['stamp']
            self.get_logger().info(f'Time reference: first image at t={self.start_time:.3f}s')
        else:
            self.start_time = 0.0
            self.get_logger().info('Time reference: 0.0 (no data)')
        
        self.real_start_time = self.get_clock().now().nanoseconds * 1e-9

    def _load_all_data(self):
        """Loads all available data (sensors and images)"""
        try:
            # 1. Load sensors (if they exist)
            sensor_file = Path(self.current_log_path) / "raw_sensors.csv"
            if sensor_file.exists():
                self._load_sensor_data(sensor_file)
                self.has_sensors = len(self.sensor_data) > 0
            else:
                self.get_logger().warning('Sensor file not found - Only images will be published')
                self.sensor_data = []
                self.has_sensors = False
            
            # 2. Load images (if they exist)
            self._load_camera_data()
            self.has_images = len(self.image_data) > 0
            
            # 3. Load lane data and perception data (for manual/change flags)
            self._load_lane_data()
            
            # 4. Save first image for calibration mode
            if self.has_images:
                self.first_image_path = self.image_data[0]['path']
                self.first_image_timestamp = self.image_data[0]['stamp']
            
            # 5. Report status
            self.get_logger().info(' DATA LOADED:')
            self.get_logger().info(f'   - Sensors: {len(self.sensor_data)} samples')
            self.get_logger().info(f'   - Images: {len(self.image_data)} files')
            
        except Exception as e:
            #self.get_logger().error(f'Error loading data: {e}')
            pass

    # =================== NORMAL REPLAY (BY TIMESTAMP) ===================

    def replay_callback(self):
        """Publishes data in timestamp order"""
        current_time = self._get_current_time()
        if current_time is None:
            return
        
        # Publish sensors (if any)
        if self.has_sensors:
            self._publish_sensors_until_time(current_time)
        
        # Publish images (if any and not in calibration mode)
        if self.has_images:
            self._publish_images_until_time(current_time)
        
        # Check if finished
        sensors_finished = not self.has_sensors or self.current_data_index >= len(self.sensor_data)
        images_finished = not self.has_images or self.current_image_index >= len(self.image_data)
        
        if sensors_finished and images_finished:
            self.get_logger().info(' Replay completed')
            self.replay_timer.cancel()

    def _publish_sensors_until_time(self, current_time):
        """Publishes sensors up to the current time"""
        while (self.current_data_index < len(self.sensor_data) and 
               self.sensor_data[self.current_data_index]['stamp'] <= current_time):
            
            data = self.sensor_data[self.current_data_index]
            
            if data['topic'] == 'odom':
                self._publish_odom(data)
            elif data['topic'] == 'imu':
                self._publish_imu(data)
            
            self.current_data_index += 1

    def _publish_images_until_time(self, current_time):
        """Publishes images up to the current time"""
        images_to_publish = []
        
        while (self.current_image_index < len(self.image_data) and 
               self.image_data[self.current_image_index]['stamp'] <= current_time):
            
            if abs(self.image_data[self.current_image_index]['stamp'] - current_time) <= 0.05:
                images_to_publish.append(self.image_data[self.current_image_index])
            
            self.current_image_index += 1
        
        if images_to_publish:
            closest = min(images_to_publish, key=lambda x: abs(x['stamp'] - current_time))
            self._publish_image_compressed(closest)

    # =================== IMAGE PUBLICATION ===================
    def _publish_image_compressed(self, img_data):
        """Publishes an image in compressed format"""
        try:
            cv_image = cv2.imread(img_data['path'])
            if cv_image is None:
                return
            
            if cv_image.shape[1] != 512 or cv_image.shape[0] != 288:
                cv_image = cv2.resize(cv_image, (512, 288), interpolation=cv2.INTER_AREA)
            
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            msg.format = 'jpeg'
            
            success, encoded = cv2.imencode('.jpg', cv_image, self.compression_params)
            if success:
                msg.data = encoded.tobytes()
                self.image_pub.publish(msg)
                
                # Publish manual, change_left and change_right based on image timestamp
                image_filename = Path(img_data['path']).name
                is_manual = self._get_is_manual_for_image(image_filename)
                change_left, change_right = self._get_change_values_for_image(image_filename)
                
                manual_msg = Bool()
                manual_msg.data = is_manual
                self.manual_pub.publish(manual_msg)
                
                change_left_msg = Bool()
                change_left_msg.data = change_left
                self.pub_change_left.publish(change_left_msg)
                
                change_right_msg = Bool()
                change_right_msg.data = change_right
                self.pub_change_right.publish(change_right_msg)
                
        except Exception as e:
            pass

    # =================== DATA LOADING ===================
    def _load_sensor_data(self, csv_file):
        """Loads sensor data"""
        self.sensor_data = []
        
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                data = {
                    'stamp': float(row['stamp']),
                    'topic': row['topic'],
                    'x': float(row['x']) if row['x'] not in ('', 'None') else None,
                    'y': float(row['y']) if row['y'] not in ('', 'None') else None,
                    'z': float(row['z']) if row['z'] not in ('', 'None') else None,
                    'qx': float(row['qx']) if row['qx'] not in ('', 'None') else None,
                    'qy': float(row['qy']) if row['qy'] not in ('', 'None') else None,
                    'qz': float(row['qz']) if row['qz'] not in ('', 'None') else None,
                    'qw': float(row['qw']) if row['qw'] not in ('', 'None') else None,
                    'vx': float(row['vx']) if row['vx'] not in ('', 'None') else None,
                    'vy': float(row['vy']) if row['vy'] not in ('', 'None') else None,
                    'vz': float(row['vz']) if row['vz'] not in ('', 'None') else None,
                    'wx': float(row['wx']) if row['wx'] not in ('', 'None') else None,
                    'wy': float(row['wy']) if row['wy'] not in ('', 'None') else None,
                    'wz': float(row['wz']) if row['wz'] not in ('', 'None') else None,
                    'ax': float(row['ax']) if row['ax'] not in ('', 'None') else None,
                    'ay': float(row['ay']) if row['ay'] not in ('', 'None') else None,
                    'az': float(row['az']) if row['az'] not in ('', 'None') else None,
                }
                self.sensor_data.append(data)
        
        self.sensor_data.sort(key=lambda x: x['stamp'])

    def _load_camera_data(self):
        """Loads data from the front camera"""
        self.image_data = []
        base_path = Path(self.current_log_path)
        
        # Search in possible folders
        possible_paths = [
            base_path / "perception" / "images",
            base_path / "perception_frontal" / "images"
        ]
        
        for image_dir in possible_paths:
            if image_dir.exists():
                self._load_images_from_dir(image_dir)
                if self.image_data:
                    break
        
        self.image_data.sort(key=lambda x: x['stamp'])

    def _load_images_from_dir(self, image_dir):
        """Loads images from a directory"""
        # Try loading from CSV first
        csv_file = image_dir.parent / "perception_data.csv"
        if csv_file.exists():
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        img_file = row.get('image_filename', '')
                        timestamp = float(row.get('timestamp', 0))
                        
                        if img_file and timestamp:
                            img_path = image_dir / img_file
                            if img_path.exists():
                                self.image_data.append({
                                    'stamp': timestamp,
                                    'path': str(img_path)
                                })
                    except:
                        continue
        
        # If there is no CSV, load by filename
        if not self.image_data:
            extensions = ['.jpg', '.jpeg', '.png']
            images = []
            for ext in extensions:
                images.extend(image_dir.glob(f'*{ext}'))
            
            images.sort()
            for i, img_path in enumerate(images):
                # Try to extract timestamp from the name
                try:
                    name = img_path.stem
                    timestamp = float(name) if name.replace('.', '').isdigit() else i * 0.1
                except:
                    timestamp = i * 0.1
                
                self.image_data.append({
                    'stamp': timestamp,
                    'path': str(img_path)
                })

    def _load_lane_data(self):
        """Loads the lane_data.csv file from robot_control."""
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
        
        # Load perception_data.csv to map image to timestamp
        perception_csv = Path(self.current_log_path) / "perception" / "perception_data.csv"
        if perception_csv.exists():
            try:
                self.perception_df = pd.read_csv(perception_csv)
                self.get_logger().info(f'Loaded perception_data.csv with {len(self.perception_df)} records')
            except Exception as e:
                self.get_logger().warning(f'Error loading perception_data.csv: {e}')
                self.perception_df = None
        else:
            self.perception_df = None

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
        is_manual_value = self.lane_data.loc[idx, 'is_manual']
        
        # Convert to bool (1 -> True, 0 -> False)
        is_manual_bool = bool(is_manual_value)
        
        return is_manual_bool
    
    def _get_change_values_for_image(self, image_filename):
        """
        Obtains the change_left and change_right values for an image
        based on its closest timestamp in lane_data.csv. 
        Searches within a range of ±20 indices around the closest match. 
        If any True value is found within that range, returns True for that side. 
        Returns: (change_left, change_right) as a tuple of booleans.
        """
        if self.lane_data is None or self.lane_data.empty:
            return False, False
        
        # Get the image timestamp
        image_timestamp = self._get_timestamp_from_image(image_filename)
        
        if image_timestamp is None:
            self.get_logger().warning(f'No timestamp found for {image_filename}')
            return False, False
        
        # Find the index of the nearest timestamp.
        idx = (self.lane_data['timestamp'] - image_timestamp).abs().idxmin()
        position_idx = self.lane_data.index.get_loc(idx)
        
        # Define the search range
        start_idx = max(0, position_idx - 20)
        end_idx = min(len(self.lane_data) - 1, position_idx + 20)
        
        # Extract the range and search for True values in a vectorized manner.
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

    # =================== UTILITIES ===================

    def _get_current_time(self):
        """Gets the current replay time"""
        if self.real_start_time is None or self.start_time is None:
            return None
        
        elapsed = (self.get_clock().now().nanoseconds * 1e-9) - self.real_start_time
        return self.start_time + (elapsed * self.replay_speed)

    # =================== SENSOR PUBLICATION ===================

    def _publish_odom(self, data):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        
        if data['x'] is not None: msg.pose.pose.position.x = data['x']
        if data['y'] is not None: msg.pose.pose.position.y = data['y']
        if data['z'] is not None: msg.pose.pose.position.z = data['z']
        
        if all(v is not None for v in [data['qx'], data['qy'], data['qz'], data['qw']]):
            msg.pose.pose.orientation.x = data['qx']
            msg.pose.pose.orientation.y = data['qy']
            msg.pose.pose.orientation.z = data['qz']
            msg.pose.pose.orientation.w = data['qw']
        
        if data['vx'] is not None: msg.twist.twist.linear.x = data['vx']
        if data['vy'] is not None: msg.twist.twist.linear.y = data['vy']
        if data['vz'] is not None: msg.twist.twist.linear.z = data['vz']
        
        if data['wx'] is not None: msg.twist.twist.angular.x = data['wx']
        if data['wy'] is not None: msg.twist.twist.angular.y = data['wy']
        if data['wz'] is not None: msg.twist.twist.angular.z = data['wz']

        msg.twist.covariance[0]  = 0.01  # Linear velocity X
        msg.twist.covariance[7]  = 0.01  # Linear velocity Y
        msg.twist.covariance[14] = 0.01  # Linear velocity Z
        msg.twist.covariance[21] = 0.01  # Angular velocity X
        msg.twist.covariance[28] = 0.01  # Angular velocity Y
        msg.twist.covariance[35] = 0.01  # Angular velocity Z (Yaw rate)
        
        self.odom_pub.publish(msg)

    def _publish_imu(self, data):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        if all(v is not None for v in [data['qx'], data['qy'], data['qz'], data['qw']]):
            msg.orientation.x = data['qx']
            msg.orientation.y = data['qy']
            msg.orientation.z = data['qz']
            msg.orientation.w = data['qw']
        
        if all(v is not None for v in [data['wx'], data['wy'], data['wz']]):
            msg.angular_velocity.x = data['wx']
            msg.angular_velocity.y = data['wy']
            msg.angular_velocity.z = data['wz']
        
        if all(v is not None for v in [data['ax'], data['ay'], data['az']]):
            msg.linear_acceleration.x = data['ax']
            msg.linear_acceleration.y = data['ay']
            msg.linear_acceleration.z = data['az']        

        msg.angular_velocity_covariance.fill(0.0)
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02

        msg.linear_acceleration_covariance.fill(0.0)
        msg.linear_acceleration_covariance[0] = 0.03
        msg.linear_acceleration_covariance[4] = 0.03 
        msg.linear_acceleration_covariance[8] = 0.03 

        msg.orientation_covariance[0] = -1.0;
        self.imu_pub.publish(msg)

def main():
    rclpy.init()
    node = DataReplayer()
    
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