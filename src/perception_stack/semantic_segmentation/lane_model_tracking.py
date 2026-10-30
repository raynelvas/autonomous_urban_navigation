#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from collections import deque

from nav_msgs.msg import Odometry
from custom_interfaces.msg import LaneModel
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import struct


class LaneTrackerEKF(Node):

    def __init__(self):
        super().__init__('lane_tracker_ekf')

        # ---------------- Parameters ----------------
        self.declare_parameters('', [
            ('q_d_lat', 0.01),
            ('q_psi', 0.01),
            ('q_kappa', 0.001),
            ('r_d_lat', 0.05),
            ('r_psi', 0.01),
            ('r_kappa', 0.01),
            ('history_length_seconds', 0.5)
        ])

        # State
        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 0.5

        self.v = 0.0
        self.omega = 0.0

        self.last_predict_time = self.get_clock().now()

        # Additional
        self.lane_width = 0.0
        self.confidence = 0.0

        # Complete history of predictions
        # Each element stores: time, x, P, v, omega, dt
        self.history = deque()
        self.max_history_time = self.get_parameter('history_length_seconds').value

        self.create_subscription(Odometry, '/odometry/local', self.cb_odom, 10)
        self.create_subscription(LaneModel, '/lane/model_raw', self.cb_lane_measurement, 10)

        self.pub_filtered = self.create_publisher(LaneModel, '/lane/model_filtered', 10)
        
        self.timer = self.create_timer(0.033, self.predict_step)

    # ==========================================================
    # ODOMETRY
    # ==========================================================
    def cb_odom(self, msg):
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    # ==========================================================
    # MODEL
    # ==========================================================
    def get_prediction_matrices(self, v, w, dt):
        F = np.array([
            [1.0,  v*dt,  0.0],
            [0.0,  1.0,   -v*dt],
            [0.0,  0.0,   1.0]
        ])

        u = np.array([[0.0],
                      [-w * dt],
                      [0.0]])

        return F, u

    # ==========================================================
    # NORMAL PREDICTION
    # ==========================================================
    def predict_step(self):
        now = self.get_clock().now()
        dt = (now - self.last_predict_time).nanoseconds / 1e9

        if dt <= 0:
            return

        self.last_predict_time = now

        F, u = self.get_prediction_matrices(self.v, self.omega, dt)

        Q = np.diag([
            self.get_parameter('q_d_lat').value,
            self.get_parameter('q_psi').value,
            self.get_parameter('q_kappa').value
        ])

        # Save previous state in history
        self.history.append({
            'time': now,
            'x': self.x.copy(),
            'P': self.P.copy(),
            'v': self.v,
            'omega': self.omega,
            'dt': dt
        })

        # Clear old history
        while self.history and \
              (now - self.history[0]['time']).nanoseconds / 1e9 > self.max_history_time:
            self.history.popleft()

        # Propagate
        self.x = F @ self.x + u
        self.P = F @ self.P @ F.T + Q

        self.publish_state()

    # ==========================================================
    # Update with real re-steering
    # ==========================================================
    def cb_lane_measurement(self, msg):
    
        self.lane_width = msg.lane_width
        self.confidence = msg.confidence

        if msg.confidence < 0.01:
            return
        
        t_cam = rclpy.time.Time.from_msg(msg.header.stamp)

        # Find the nearest state BEFORE the image
        past_states = [h for h in self.history if h['time'] <= t_cam]

        if not past_states:
            # No hay historial suficiente
            self.apply_update(msg)
            return

        # Take the last state before the image.
        base_state = past_states[-1]

        self.x = base_state['x'].copy()
        self.P = base_state['P'].copy()

        # ---------------- Real-time update ----------------
        self.apply_update(msg)

        # ---------------- RE-PROPAGATION ----------------
        future_states = [h for h in self.history if h['time'] > t_cam]

        Q = np.diag([
            self.get_parameter('q_d_lat').value,
            self.get_parameter('q_psi').value,
            self.get_parameter('q_kappa').value
        ])

        for h in future_states:
            F, u = self.get_prediction_matrices(h['v'], h['omega'], h['dt'])
            self.x = F @ self.x + u
            self.P = F @ self.P @ F.T + Q

    # ==========================================================
    # UPDATE EKF
    # ==========================================================
    def apply_update(self, msg):

        z = np.array([[msg.d_lat],
                      [msg.yaw],
                      [msg.curvature]])

        H = np.eye(3)

        R = np.diag([
            self.get_parameter('r_d_lat').value,
            self.get_parameter('r_psi').value,
            self.get_parameter('r_kappa').value
        ]) / (msg.confidence + 1e-4)

        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

    # ==========================================================
    # PUBLICATION
    # ==========================================================
    def publish_state(self):

        m = LaneModel()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_footprint"

        m.d_lat = float(self.x[0, 0])
        m.yaw = float(self.x[1, 0])
        m.curvature = float(self.x[2, 0])
        m.lane_width = self.lane_width
        m.confidence = self.confidence

        self.pub_filtered.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = LaneTrackerEKF()
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