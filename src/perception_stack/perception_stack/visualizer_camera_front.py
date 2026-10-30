#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from custom_interfaces.msg import LaneModel, DetectionArray, Detection
from cv_bridge import CvBridge
import time
from collections import defaultdict
from custom_interfaces.msg import PixelPoint
from std_msgs.msg import Bool
from message_filters import Subscriber, ApproximateTimeSynchronizer
from custom_interfaces.msg import PixelCluster, PixelClusterArray

class CombinedVisualizerNode(Node):
    def __init__(self):
        super().__init__('combined_visualizer')
        
        self.bridge = CvBridge()
        
        # Colores por clase (BGR)
        self.class_colors = {
            0: (255, 0, 0),     #0: person - Blue
            1: (0, 255, 0),     #1: bicycle - Green
            2: (0, 0, 255),     #2: car - Red
            3: (255, 255, 0),   #3: motorcycle - Cyan
            4: (255, 0, 255),   #4: bus - Magenta
            5: (0, 255, 255),   #5: truck - Yellow
            6: (128, 0, 128),   #6: speed_bump - Purple
            7: (255, 128, 0),   #7: crosswalk - Orange
            8: (0, 128, 255),   #8: speed_bump_signage - Light Blue
            9: (128, 128, 0)    #9: crosswalk_signage - Olive Green
        }

        self.class_names = {
            0: "person",
            1: "bicycle", 
            2: "car",
            3: "motorcycle",
            4: "bus",
            5: "truck",
            6: "speed_bump",
            7: "crosswalk",
            8: "speed_bump_signage",
            9: "crosswalk_signage"
        }

        self.current_image = None
        self.current_lane_pixels = None
        self.current_lane_selection_pixels = None # Green dots result of segmentation network
        self.current_detections = None #Cyan dots rail pattern
        
        # Asynchronous data
        self.lane_left_pixels = None
        self.lane_right_pixels = None

        # --- METRICS ---
        self.last_fps_time = time.time()
        self.frame_count = 0
        self.fps = 0
        
        self.manual = False
        self.recording = False

        self.debug_clusters = []  # Ready to store received debug clusters

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        
       # --- SYNCHRONIZED SUBSCRIPTIONS (important topics) --- 
        self.image_sub = Subscriber(self, CompressedImage, '/image_raw/compressed', qos_profile=qos)
        self.detection_sub = Subscriber(self, DetectionArray, '/detection/results', qos_profile=qos)
        self.lane_candidates_sub = Subscriber(self, PixelPoint, '/lane/pixel_candidates', qos_profile=qos)
        self.lane_selection_sub = Subscriber(self, PixelPoint, '/lane/ipm_inverse_pixel_points', qos_profile=qos)
        self.lane_debug_clusters_sub = Subscriber(self, PixelClusterArray, '/lane/pixel_debug_clusters', qos_profile=qos)
        
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.detection_sub, self.lane_candidates_sub, self.lane_selection_sub],#, self.lane_debug_clusters_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)
        
        # --- ASYNCHRONOUS SUBSCRIPTIONS ---
        self.create_subscription(PixelPoint, '/lane/ipm_inverse_pixel_points_left', self.lane_left_callback, qos_profile=qos)
        self.create_subscription(PixelPoint, '/lane/ipm_inverse_pixel_points_right', self.lane_right_callback, qos_profile=qos)
        
        # Status Subscriptions
        self.create_subscription(Bool, '/manual', self.manual_mode_cb, 10)
        self.create_subscription(Bool, '/record', self.record_cb, 10)
        
        # Publisher
        self.viz_pub = self.create_publisher(Image, '/debug/camera_front', 10)

    # --- SYNCHRONIZED CALLBACK ---
    def sync_callback(self, img_msg: CompressedImage, det_msg: DetectionArray,
                      lane_msg: PixelPoint, selection_msg: PixelPoint):#, debug_clusters_msg: PixelClusterArray):
        """Callback que recibe todos los tópicos sincronizados"""
        try:
            # Process image
            np_arr = np.frombuffer(img_msg.data, np.uint8)
            self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Process detections
            if det_msg and hasattr(det_msg, 'detections'):
                self.current_detections = det_msg.detections
            else:
                self.current_detections = None
            
            # Process lane candidates
            if lane_msg and len(lane_msg.u) > 0:
                self.current_lane_pixels = list(zip(lane_msg.u, lane_msg.v))
            else:
                self.current_lane_pixels = None
            
            # Process lane selection
            if selection_msg and len(selection_msg.u) > 0:
                self.current_lane_selection_pixels = list(zip(selection_msg.u, selection_msg.v))
            else:
                self.current_lane_selection_pixels = None
            
            """if debug_clusters_msg and hasattr(debug_clusters_msg, 'clusters'):
                self.debug_clusters = debug_clusters_msg.clusters
            else:
                self.debug_clusters = None"""
            
            # Render if we have image
            if self.current_image is not None:
                self.process_and_publish()
                self.update_fps()
                
        except Exception as e:
            self.get_logger().error(f"Error en sync_callback: {e}")

    # --- ASYNCHRONOUS  CALLBACKS ---
    def lane_left_callback(self, msg: PixelPoint):
        """Asynchronous callback for left lane"""
        if msg and len(msg.u) > 0:
            self.lane_left_pixels = list(zip(msg.u, msg.v))
        else:
            self.lane_left_pixels = None

    def lane_right_callback(self, msg: PixelPoint):
        """Asynchronous callback for right lane"""
        if msg and len(msg.u) > 0:
            self.lane_right_pixels = list(zip(msg.u, msg.v))
        else:
            self.lane_right_pixels = None

    def manual_mode_cb(self, msg: Bool):
        self.manual = msg.data

    def record_cb(self, msg: Bool):
        self.recording = msg.data

    # --- PROCESSING LOGIC ---
    def process_and_publish(self):
        if self.current_image is None:
            return
        
        try:
            display_image = self.current_image.copy()
            
            # 1. Draw the overlay of the selected lane (Filtered)
            if self.current_lane_selection_pixels is not None:
                display_image = self.draw_lane_overlay(display_image, self.current_lane_selection_pixels, color=(235, 183, 0))
            
            # 2. Draw left lane
            if self.lane_left_pixels is not None:
                display_image = self.draw_lane_overlay(display_image, self.lane_left_pixels, color=(255, 0, 180), alpha=0.4)
            
            # 3. Draw right lane
            if self.lane_right_pixels is not None:
                display_image = self.draw_lane_overlay(display_image, self.lane_right_pixels, color=(0, 255, 255), alpha=0.4)

            # 4. Draw segmentation candidate pixels (Green)
            if self.current_lane_pixels is not None:
                display_image = self.draw_pixels(display_image, self.current_lane_pixels, (0, 255, 0))

            """if self.debug_clusters is not None:
                for cluster in self.debug_clusters:
                    color = (int(cluster.r), int(cluster.g), int(cluster.b))
                    
                    display_image = self.draw_pixels(display_image, list(zip(cluster.u, cluster.v)), color=color)"""
            
            # 5. Draw tracking detections with bounding boxes and text
            if self.current_detections is not None:
                for det in self.current_detections:
                    color = self.class_colors.get(det.class_id, (255, 255, 255))
                    label_class = self.class_names.get(det.class_id, f"class_{det.class_id}")
                    
                    cv2.rectangle(display_image, (det.u1, det.v1), (det.u2, det.v2), color, 2)
                    
                    label = f"{label_class} {det.track_id}"
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.5
                    thickness = 1
                    
                    (w, h), baseline = cv2.getTextSize(label, font, font_scale, thickness)
                    
                    label_y = det.v1 - 5 
                    
                    # Check if the label would come off the image (above)
                    if det.v1 - h - baseline - 5 < 0:
                        # Place the label at the bottom of the bounding box
                        label_y = det.v2 + h + baseline 
                        

                        if label_y > display_image.shape[0]:
                            # If it also comes out at the bottom, adjust within the image
                            label_y = display_image.shape[0] - 5
                            rect_y1 = label_y - h - baseline
                            rect_y2 = label_y + baseline
                        else:
                            rect_y1 = det.v2
                            rect_y2 = det.v2 + h + baseline + 5
                        
                        cv2.rectangle(display_image, 
                                    (det.u1, rect_y1), 
                                    (det.u1 + w, rect_y2), 
                                    color, -1)
                        
                        cv2.putText(display_image, label, (det.u1, label_y), 
                                font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)
                    else:
                        # Place the label at the top of the bounding box
                        cv2.rectangle(display_image, 
                                    (det.u1, det.v1 - h - baseline - 5), 
                                    (det.u1 + w, det.v1), 
                                    color, -1)
                        cv2.putText(display_image, label, (det.u1, det.v1 - 5), 
                                font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)
            
            # 6. Information Overlay
            display_image = self.add_info_overlay(display_image)
            
            # Post final image for debugging
            viz_msg = self.bridge.cv2_to_imgmsg(display_image, 'bgr8')
            viz_msg.header.stamp = self.get_clock().now().to_msg()
            self.viz_pub.publish(viz_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in visualization processing: {e}")

    def draw_lane_overlay(self, image, pixels, color=(235, 183, 0), alpha=0.6):
        """Draws a filled polygon for the lane with alpha blending based on the provided pixels"""
        if pixels is None or len(pixels) < 2:
            return image
        
        height, width = image.shape[:2]
        points_by_v = defaultdict(list)
        
        for u, v in pixels:
            v_int, u_int = int(v), int(u)
            if 0 <= v_int < height:
                u_clamped = max(0, min(u_int, width - 1))
                points_by_v[v_int].append(u_clamped)
        
        if not points_by_v: 
            return image
        
        sorted_vs = sorted(points_by_v.keys())
        min_v_detected = sorted_vs[0]
        left_side, right_side = [], []
        last_min_u = min(points_by_v[min_v_detected])
        last_max_u = max(points_by_v[min_v_detected])
        
        for v in range(min_v_detected, height):
            if v in points_by_v:
                curr_min, curr_max = min(points_by_v[v]), max(points_by_v[v])
                if last_min_u <= 1: 
                    curr_min = 0
                if last_max_u >= width - 2: 
                    curr_max = width - 1
                last_min_u, last_max_u = curr_min, curr_max
            else:
                if last_min_u < 15: 
                    last_min_u = 0
                if last_max_u > width - 15: 
                    last_max_u = width - 1
            left_side.append([last_min_u, v])
            right_side.append([last_max_u, v])
        
        polygon_points = np.array(left_side + right_side[::-1], dtype=np.int32)
        
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, [polygon_points], 255)
        
        overlay = np.full_like(image, color, dtype=np.uint8)
        mask_3ch = mask[:, :, np.newaxis] / 255.0
        
        image = (image * (1 - alpha * mask_3ch) + overlay * (alpha * mask_3ch)).astype(np.uint8)
        return image
    
    def draw_pixels(self, image, pixels, color=(255, 255, 0)):
        if pixels is None or len(pixels) == 0: 
            return image
        pixel_color = (color[2], color[1], color[0])
        for u, v in pixels:
            x, y = int(u), int(v)
            if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                cv2.circle(image, (x, y), 3, pixel_color, -1)
        return image

    def add_info_overlay(self, img):
        img_height, img_width = img.shape[:2]

        # Text
        mode_text = "Manual" if self.manual else "Autonomo"
        modo_recording = "Grabando" if self.recording else None
        
        # Configuration
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale_fps = 0.5
        font_scale_mode = 0.55
        
        # Get dimensions
        (w_mode, h_mode), _ = cv2.getTextSize(mode_text, font, font_scale_mode, 2)
        (w_record, h_record), _ = cv2.getTextSize(modo_recording, font, font_scale_mode, 2) if modo_recording else ((0, 0), 0)

        spacing = 8
        padding = 6
        rect_width = spacing + w_mode + (padding * 2)
        rect_height = h_mode + (padding * 3) + (h_record) if modo_recording else h_mode + (padding * 2) + (h_record)
        
        rect_x1 = (img_width - rect_width) // 2
        rect_x2 = rect_x1 + rect_width
        rect_y1 = 0
        rect_y2 = rect_y1 + rect_height
        
        overlay = img.copy()
        cv2.rectangle(overlay, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 0, 0), -1)
        img = cv2.addWeighted(overlay, 0.45, img, 0.55, 0)

        # FPS
        # cv2.putText(img, fps_text, (rect_x1 + padding, rect_y1 + padding + h_fps - 2), 
        #            font, font_scale_fps, (200, 200, 200), 1, cv2.LINE_AA)
        
        mode_color = (100, 255, 100) if not self.manual else (255, 150, 150)
        mode_x = rect_x1 + padding + spacing
        cv2.putText(img, mode_text, (mode_x, rect_y1 + padding + h_mode - 2), 
                    font, font_scale_mode, mode_color, 2, cv2.LINE_AA)
        
        if modo_recording:
            record_color = (255, 255, 255)  
            record_x = rect_x1 + (rect_width - w_record) // 2
            cv2.putText(img, modo_recording, (record_x, rect_y1 + padding*2 + h_mode + h_record - 2), 
                        font, font_scale_fps, record_color, 1, cv2.LINE_AA)
        
        return img
    
    def update_fps(self):
        self.frame_count += 1
        now = time.time()
        if now - self.last_fps_time >= 1.0:
            self.fps = self.frame_count / (now - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = now

def main(args=None):
    rclpy.init(args=args)
    node = CombinedVisualizerNode()
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