#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import math
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry

from std_msgs.msg import Int32
from custom_interfaces.msg import LaneModel

class LaneControllerStep6(Node):

    def __init__(self):
        super().__init__('lane_controller_step6')

        # ===================== SUBS =====================
        self.create_subscription(Odometry, '/odometry/local', self.odom_cb, 10)

        self.create_subscription(Bool, '/emergency', self.emergency_cb, 10)
        self.create_subscription(Bool, '/manual', self.manual_cb, 10)

        #self.create_subscription(Bool, '/active/planner', self.planner_active_cb, 10)
        self.create_subscription(Bool, '/goal_reached', self.goal_reached_cb, 10)

        self.create_subscription(Float32, '/alpha/vision', self.vision_alpha_cb, 10)
        self.create_subscription(Bool, '/active/vision', self.vision_active_cb, 10)

        self.create_subscription(LaneModel, '/lane/model_filtered', self.lane_model_cb, 10)

        self.goal_reached = False

        self.alpha_vision = None
        self.active_vision = None

        # ===================== PUB ======================
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # =================== PARAMS =====================
        self.declare_parameter('v_max', 1.5)
        self.declare_parameter('w_max', 0.3)
        self.declare_parameter('max_lin_accel', 0.06)

        # LANE (control geométrico)
        self.declare_parameter('lookahead', 2.0)

        self.declare_parameter('k_lat', 0.55) #Qué tan rápido vuelves al carril cuando estás desplazado
        self.declare_parameter('k_head', 1.5) #Qué tan agresivo alineas el robot con la dirección del carril
        self.declare_parameter('k_curv', 1.0) #Anticipación de curva
        self.declare_parameter('beta', 2.0) #Cuánto reduces velocidad lineal en curva

        # =================== STATE ======================
        self.mode = 'BASE'          # BASE → TRANSITION → LANE
        self.alpha = 0.0

        self.start_time = time.time()

        self.e_lat = None
        self.e_head = None
        self.curvature = 0.0

        self.omega_lane = 0.0
        self.omega_lane_orientation = 0.0
        self.active_lane = False

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.v_cmd = 0.0
        self.w_cmd = 0.0

        self.e_lane_prev = 0.0
        self.e_lane_dot = 0.0
        self.deriv_filter = 0.85

        self.emergency = False
        self.manual = False

        self.last_time = time.time()
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('LaneController Step6 FINAL READY')
    
    # ============ CONTROL DE VISION =====================
    def vision_alpha_cb(self, msg):
        self.alpha_vision = msg.data

    def vision_active_cb(self, msg):
        self.active_vision = msg.data

    # ================= CALLBACKS =================
    def lane_cb(self, msg): self.omega_lane = msg.data
    def lane_orientation_cb(self, msg): self.omega_lane_orientation = msg.data
    def lane_active_cb(self, msg): self.active_lane = msg.data

    def manual_cb(self, msg): self.manual = msg.data
    def emergency_cb(self, msg): self.emergency = msg.data

    def odom_cb(self, msg):
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

    def goal_reached_cb(self, msg): self.goal_reached = msg.data


    def lane_model_cb(self, msg):
        if msg.confidence < 0.01: 
            self.e_lat = 0.0
            self.e_head = 0.0
            self.curvature = 0.0
            return

        self.e_lat = msg.d_lat
        self.e_head = msg.yaw
        self.curvature = msg.curvature


    # ================= CONTROL LOOP =================
    def control_loop(self):
        now = time.time()
        dt = max(1e-3, now - self.last_time)
        self.last_time = now

        # ---------- GOAL REACHED ----------
        """if self.goal_reached or not self.planner_active:
            self.reset_controller()
            self.publish_zero()
            return"""

        # ---------- MANUAL ----------
        if self.manual:
            return

        # ---------- EMERGENCY ----------
        if self.emergency:
            self.reset_controller()
            self.publish_zero()
            return

        v, w = self.lane_control(dt)
        self.publish_cmd(v, w)

    # ================= LANE CONTROL (GEOMÉTRICO) =================
    def lane_control(self, dt):
        if self.e_lat is None or self.e_head is None:
            return self.limit_accel(0.2, dt), 0.0

        # Parámetros
        k_lat  = self.get_parameter('k_lat').value * (1 + 1.0*self.linear_velocity)
        k_head = self.get_parameter('k_head').value * (1 + 1.5*self.linear_velocity)
        k_curv = self.get_parameter('k_curv').value * (1 + 1.05*self.linear_velocity)
        L      = self.get_parameter('lookahead').value

        # Velocidad de referencia (NO la medida)
        v_ref = max(self.v_cmd, 0.05)
        

        # --- Términos del control ---
        w_heading = k_head * v_ref * self.e_head / L

        w_lat = k_lat * v_ref * self.e_lat / (L*L + self.e_lat*self.e_lat)

        w_curv = k_curv * v_ref * self.curvature

        w = w_heading + w_lat + w_curv
        #self.get_logger().info(f"orientacion:{w_heading:.2f} lateral:{w_lat:.2f} curva: {w_curv:.2f} resultante: {w}")

        # Saturación física
        w = np.clip(
            w,
            -self.get_parameter('w_max').value,
            self.get_parameter('w_max').value
        )

        # Velocidad lineal adaptativa a curvatura
        v_max = self.get_parameter('v_max').value

        if self.active_vision:
            v_max = v_max*self.alpha_vision

        beta  = self.get_parameter('beta').value
        v_des = v_max / (1 + beta * abs(self.curvature))

        v = self.limit_accel(v_des, dt)
        return v, w


    # ================= UTIL =================
    def limit_accel(self, v_des, dt):
        a = self.get_parameter('max_lin_accel').value
        dv = np.clip(v_des - self.v_cmd, -a * dt, a * dt)
        self.v_cmd += dv
        return self.v_cmd

    def publish_cmd(self, v, w):
        self.v_cmd = v
        self.w_cmd = w
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

    def publish_zero(self):
        self.pub_cmd.publish(Twist())

    def reset_controller(self):
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.alpha = 0.0
        self.mode = 'BASE'
        self.start_time = time.time()
        self.e_lane_prev = 0.0
        self.e_lane_dot = 0.0

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = LaneControllerStep6()
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