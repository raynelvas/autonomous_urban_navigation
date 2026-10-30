#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from custom_interfaces.msg import PixelPoint, DetectionArray, Detection
from cv_bridge import CvBridge
import time
from collections import defaultdict
import math
from std_msgs.msg import Bool

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

        self.current_detections = None 
        
        self.current_image = None
        self.current_objects = None
        self.current_lane_pixels = None       # Green dots result of segmentation network
        self.current_centerline_pixels = None  # Cyan dots rail pattern

        # Asynchronous data
        self.lane_left_pixels = None
        self.lane_right_pixels = None
        
        # --- METRICS ---
        self.last_fps_time = time.time()
        self.frame_count = 0
        self.fps = 0.0
        
        self.manual = False
        self.recording = False

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        self.create_subscription(Image, '/image_raw', self.image_callback, qos)
        self.create_subscription(PixelPoint, '/lane/pixel_candidates', self.lane_callback, 10)
        self.create_subscription(PixelPoint, '/lane/ipm_inverse_pixel_points', self.centerline_callback, 10)
        self.create_subscription(Bool, '/manual', self.manual_mode_cb, 10)
        self.create_subscription(Bool, '/record', self.record_cb, 10)
        self.det_sub = self.create_subscription(DetectionArray, '/detection/results', self.detections_callback, 10)
        self.create_subscription(PixelPoint, '/lane/ipm_inverse_pixel_points_left', self.lane_left_callback, qos_profile=qos)
        self.create_subscription(PixelPoint, '/lane/ipm_inverse_pixel_points_right', self.lane_right_callback, qos_profile=qos)

        # Publisher
        self.viz_pub = self.create_publisher(Image, '/debug/camera_front', 10)

    # --- CALLBACKS ---
    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_and_publish()

    def lane_callback(self, msg): self.current_lane_pixels = list(zip(msg.u, msg.v))
    def centerline_callback(self, msg): self.current_centerline_pixels = list(zip(msg.u, msg.v))

    def lane_left_callback(self, msg: PixelPoint):
        self.lane_left_pixels = list(zip(msg.u, msg.v))
        
    def lane_right_callback(self, msg: PixelPoint):
        self.lane_right_pixels = list(zip(msg.u, msg.v))
        
    def detections_callback(self, msg):
        self.current_detections = msg.detections
        
    def manual_mode_cb(self, msg: Bool):
        self.manual = msg.data

    def record_cb(self, msg: Bool):
        self.recording = msg.data

    # --- PROCESSING LOGIC ---
    def process_and_publish(self):
        if self.current_image is None: return
        
        try: 
            vis_img = self.current_image.copy()

            # 1. Draw the overlay of the selected lane (Filtered)
            if self.current_centerline_pixels:
                vis_img = self.draw_lane_selection_overlay(vis_img,self.current_centerline_pixels, color=(235, 183, 0))

            # 2. Draw left lane
            if self.lane_left_pixels:
                vis_img = self.draw_lane_selection_overlay(vis_img, self.lane_left_pixels, color=(255, 0, 180), alpha=0.4)

            # 3. Draw right lane
            if self.lane_right_pixels:
                vis_img = self.draw_lane_selection_overlay(vis_img, self.lane_right_pixels, color=(0, 255, 255), alpha=0.4)

            # 4. Draw segmentation candidate pixels (Green)
            if self.current_lane_pixels:
                for u, v in self.current_lane_pixels:
                    if v < vis_img.shape[0] and u < vis_img.shape[1]:
                        cv2.circle(vis_img, (int(u), int(v)), 2, (0, 255, 0), -1)            

            # 4. Draw tracking detections with bounding boxes and text
            if self.current_detections is not None:
                for det in self.current_detections:
                    color = self.class_colors.get(det.class_id, (255, 255, 255))
                    label_class = self.class_names.get(det.class_id, f"class_{det.class_id}")
                    
                    cv2.rectangle(vis_img, (det.u1, det.v1), (det.u2, det.v2), color, 2)
                    
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
                        
                        if label_y > vis_img.shape[0]:
                            # If it also comes out at the bottom, adjust within the image
                            label_y = vis_img.shape[0] - 5
                            rect_y1 = label_y - h - baseline
                            rect_y2 = label_y + baseline
                        else:
                            rect_y1 = det.v2
                            rect_y2 = det.v2 + h + baseline + 5
                        
                        cv2.rectangle(vis_img, 
                                    (det.u1, rect_y1), 
                                    (det.u1 + w, rect_y2), 
                                    color, -1)
                        
                        cv2.putText(vis_img, label, (det.u1, label_y), 
                                font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)
                    else:
                        # Place the label at the top of the bounding box
                        cv2.rectangle(vis_img, 
                                    (det.u1, det.v1 - h - baseline - 5), 
                                    (det.u1 + w, det.v1), 
                                    color, -1)
                        cv2.putText(vis_img, label, (det.u1, det.v1 - 5), 
                                font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)

            # 6. Information Overlay
            vis_img = self.add_info_overlay(vis_img)
            
            # Post final image for debugging
            self.viz_pub.publish(self.bridge.cv2_to_imgmsg(vis_img, "bgr8"))
            self.update_fps()

        except Exception as e:
            self.get_logger().error(f"Error in visualization processing: {e}")

    def add_info_overlay(self, img):
        img_height, img_width = img.shape[:2]
        
        # Text
        fps_text = f"{self.fps:.1f}"
        mode_text = "Manual" if self.manual else "Autonomo"
        modo_recording = "Grabando" if self.recording else None
        
        # Configuration
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale_fps = 0.5
        font_scale_mode = 0.55
        
        # Get dimensions
        (w_fps, h_fps), _ = cv2.getTextSize(fps_text, font, font_scale_fps, 1)
        (w_mode, h_mode), _ = cv2.getTextSize(mode_text, font, font_scale_mode, 2)
        (w_record, h_record), _ = cv2.getTextSize(modo_recording, font, font_scale_mode, 2) if modo_recording else ((0, 0), 0)
        
        spacing = 8
        padding = 6
        rect_width = w_fps + spacing + w_mode + (padding * 2)
        rect_height = max(h_fps, h_mode) + (padding * 3) + (h_record) if modo_recording else max(h_fps, h_mode) + (padding * 2) + (h_record)
        
        rect_x1 = (img_width - rect_width) // 2
        rect_x2 = rect_x1 + rect_width
        rect_y1 = 0
        rect_y2 = rect_y1 + rect_height
        
        overlay = img.copy()
        cv2.rectangle(overlay, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 0, 0), -1)
        img = cv2.addWeighted(overlay, 0.45, img, 0.55, 0)
        
        # FPS
        cv2.putText(img, fps_text, (rect_x1 + padding, rect_y1 + padding + h_fps - 2), 
                    font, font_scale_fps, (200, 200, 200), 1, cv2.LINE_AA)
        
        mode_color = (100, 255, 100) if not self.manual else (255, 150, 150)
        mode_x = rect_x1 + padding + w_fps + spacing
        cv2.putText(img, mode_text, (mode_x, rect_y1 + padding + h_mode - 2), 
                    font, font_scale_mode, mode_color, 2, cv2.LINE_AA)
        
        if modo_recording:
            record_color = (255, 255, 255)  
            record_x = rect_x1 + (rect_width - w_record) // 2
            cv2.putText(img, modo_recording, (record_x, rect_y1 + padding*2 + h_mode + h_record - 2), 
                        font, font_scale_fps, record_color, 1, cv2.LINE_AA)
        
        return img
    
    def draw_lane_selection_overlay(self, image, pixels, color=(235, 183, 0), alpha=0.6):
        """Draws a filled polygon for the lane with alpha blending based on the provided pixels."""
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