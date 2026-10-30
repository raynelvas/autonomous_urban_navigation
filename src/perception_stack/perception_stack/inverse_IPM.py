#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import json
import math

from custom_interfaces.msg import LaneModel, PixelPoint

class IPMInverseNode(Node):
    def __init__(self):
        super().__init__('ipm_inverse_node')
        
        # ============ Declare parameters in the launch ============
        self.declare_parameter('config_path', 'camera_calibration_512x288.json')

        calibration_path = self.get_parameter('config_path').get_parameter_value().string_value
        # =============================================

        try:
            with open(calibration_path, 'r') as f:
                calib = json.load(f)

            self.cam_z = calib["camera_z"]
            self.cam_x = calib["camera_x"]
            self.cam_y = calib["camera_y"]
            self.roll = float(calib['camera_roll'])
            self.fx = float(calib['intrinsics']['fx'])
            self.fy = float(calib['intrinsics']['fy'])
            self.cx = float(calib['intrinsics']['cx'])
            self.cy = float(calib['intrinsics']['cy'])
            
            self.img_width = 512
            self.img_height = 288

        except Exception as e:
            self.get_logger().error(f"Calibration failed to load: {e}")
            rclpy.shutdown()
            return

        # =================== HOMOGRAPHY CONSTRUCTION ===================
        K = np.array([
            [self.fx,  0.0,     self.cx],
            [0.0,      self.fy,     self.cy],
            [0.0,      0.0,     1.0]
        ])
        
        R_align = np.array([
            [0.0, -1.0,  0.0],
            [0.0,  0.0, -1.0],
            [1.0,  0.0,  0.0]
        ])

        cp = math.cos(self.roll)
        sp = math.sin(self.roll)
        R_pitch = np.array([
            [1.0,  0.0,  0.0],
            [0.0,   cp,   sp],
            [0.0,  -sp,   cp]
        ])
        
        rotation_matrix = R_pitch @ R_align
        cam_position = np.array([self.cam_x, self.cam_y, self.cam_z])
        translation = -rotation_matrix @ cam_position

        self.H = K @ np.column_stack((rotation_matrix[:, 0], 
                                      rotation_matrix[:, 1], 
                                      translation))

        # =================== COMMUNICATIONS ===================
        self.create_subscription(LaneModel, '/lane/model_filtered', self.lane_model_callback, 10)
        self.pub_pixel_points = self.create_publisher(PixelPoint, '/lane/ipm_inverse_pixel_points', 10)

        self.create_subscription(LaneModel, '/lane/model_left', self.lane_model_left_callback, 10)
        self.pub_pixel_points_left = self.create_publisher(PixelPoint, '/lane/ipm_inverse_pixel_points_left', 10)

        self.create_subscription(LaneModel, '/lane/model_right', self.lane_model_right_callback, 10)
        self.pub_pixel_points_right = self.create_publisher(PixelPoint, '/lane/ipm_inverse_pixel_points_right', 10)

    def project_points(self, xs, ys):
        """Projects world points (x, y) to image pixels (u, v)"""
        n_points = xs.shape[0]
        input_pts = np.ones((3, n_points))
        input_pts[0, :] = xs
        input_pts[1, :] = ys

        projected = self.H @ input_pts
        w = projected[2, :]
        
        mask_valid = w > 0.1
        u_pixels = projected[0, mask_valid] / w[mask_valid]
        v_pixels = projected[1, mask_valid] / w[mask_valid]

        return u_pixels, v_pixels

    def process_lane_model(self, msg: LaneModel, publisher):
        """Process a lane model and publish the projected points"""
        if msg.confidence < 0.01:
            out_msg = PixelPoint()
            out_msg.header = msg.header
            out_msg.u = []
            out_msg.v = []
            publisher.publish(out_msg)
            return

        # 1. Model parameters
        d_lat = msg.d_lat
        tan_yaw = math.tan(msg.yaw)
        c2 = 0.5 * msg.curvature
        half_width = msg.lane_width / 2.0

        # Generate distances (X) logarithmically
        xs = np.logspace(0, np.log10(15.0), 250)

        # 2. Generate LEFT Line (y = y_center + half_width)
        ys_left = c2 * (xs**2) + tan_yaw * xs + (d_lat + half_width)
        u_left, v_left = self.project_points(xs, ys_left)

        # 3. Generate RIGHT Line (y = y_center - half_width)
        ys_right = c2 * (xs**2) + tan_yaw * xs + (d_lat - half_width)
        u_right, v_right = self.project_points(xs, ys_right)

        # 4. Filter points within the image
        final_u = []
        final_v = []

        def collect_valid(u_pts, v_pts):
            for u, v in zip(u_pts, v_pts):
                if 0 <= u < self.img_width and 0 <= v < self.img_height:
                    final_u.append(int(u))
                    final_v.append(int(v))

        collect_valid(u_left, v_left)
        collect_valid(u_right, v_right)

        # 5. Publish
        out_msg = PixelPoint()
        out_msg.header = msg.header
        out_msg.u = final_u
        out_msg.v = final_v
        publisher.publish(out_msg)

    def lane_model_callback(self, msg: LaneModel):
        self.process_lane_model(msg, self.pub_pixel_points)

    def lane_model_left_callback(self, msg: LaneModel):
        self.process_lane_model(msg, self.pub_pixel_points_left)

    def lane_model_right_callback(self, msg: LaneModel):
        self.process_lane_model(msg, self.pub_pixel_points_right)

def main(args=None):
    rclpy.init(args=args)
    node = IPMInverseNode()
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