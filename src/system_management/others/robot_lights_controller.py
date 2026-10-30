#!/usr/bin/env python3
from custom_interfaces.msg import LaneModel
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from collections import deque

class RobotLightsController(Node):
    def __init__(self):
        super().__init__('robot_lights_controller')

        # --- Control Parameters ---
        self.declare_parameter('vel_threshold', 0.05)    # Considered "stopped" (m/s)
        self.declare_parameter('brake_delta', 0.08)      # Sensitivity: braking response
        self.declare_parameter('stop_timeout', 3.0)      # Seconds of steady red light when stopped
        self.declare_parameter('window_size', 5)         # EKF velocity smoothing

        self.threshold = self.get_parameter('vel_threshold').value
        self.brake_delta = self.get_parameter('brake_delta').value
        self.stop_timeout = self.get_parameter('stop_timeout').value
        
        # --- Calculation Variables ---
        self.vel_buffer = deque(maxlen=self.get_parameter('window_size').value)
        self.vel_buffer.append(0.0)
        self.v_prev = 0.0
        self.reference_v = 0.0 
        
        # --- Publishers ---
        self.pub_stop = self.create_publisher(Bool, '/light/stop', 10)
        self.pub_left = self.create_publisher(Bool, '/light/left', 10)
        self.pub_right = self.create_publisher(Bool, '/light/right', 10)

        # --- Subscribers ---
        self.create_subscription(Odometry, '/odometry/local', self.odom_callback, 10)
        self.create_subscription(LaneModel, '/lane/model_filtered', self.lane_cb, 10)
        self.create_subscription(LaneModel, '/lane/model_left', self.lane_model_left_callback, 10)
        self.create_subscription(LaneModel, '/lane/model_right', self.lane_model_right_callback, 10)

        self.d_lat = 0.0
        self.conf_last_left = 0.0
        self.conf_last_right = 0.0
        
        # Autonomous maneuvering states
        self.left_maneuver = False
        self.right_maneuver = False
        self.change_n = 0

        # --- Light State Variables ---
        self.is_moving = False
        self.stop_start_time = None
        self.last_blink_time = self.get_clock().now()
        self.blink_state = False

        # Timer
        self.timer = self.create_timer(0.1, self.update_logic_loop)

    def odom_callback(self, msg):
        raw_vel = abs(msg.twist.twist.linear.x)
        self.vel_buffer.append(raw_vel)
    
    def lane_cb(self, msg):
        self.d_lat = msg.d_lat
    
    def lane_model_left_callback(self, msg: LaneModel):
        conf_left = float(msg.confidence)
        # Left Turn Maneuver
        if self.conf_last_left != 0.0 and conf_left == 0.0:
            self.left_maneuver = True
            self.right_maneuver = False
            self.change_n = 0
        self.conf_last_left = conf_left

    def lane_model_right_callback(self, msg: LaneModel):
        conf_right = float(msg.confidence)
        # Right Turn Maneuver
        if self.conf_last_right != 0.0 and conf_right == 0.0:
            self.right_maneuver = True
            self.left_maneuver = False
            self.change_n = 0
        self.conf_last_right = conf_right

    def update_logic_loop(self):
        if not self.vel_buffer:
            return
        
        now = self.get_clock().now()
        v_current = sum(self.vel_buffer) / len(self.vel_buffer)

        # Local output variables
        out_brake = False
        out_left = False
        out_right = False

        # -------------------------------------------------
        # LOGIC 1: FLASHING CLOCK (For steady-state or directional)
        # -------------------------------------------------
        elapsed_blink = (now - self.last_blink_time).nanoseconds / 1e9
        if elapsed_blink >= 0.5:  
            self.blink_state = not self.blink_state
            self.last_blink_time = now

        # -------------------------------------------------
        # LOGIC 2: DIRECTIONALS VIA LANE CHANGE
        # -------------------------------------------------
        if self.left_maneuver:
            if self.d_lat > 1.5:     
                out_left = self.blink_state
                self.change_n = 0 # Reset count if successfully crossing the line
            else:
                out_left = self.blink_state
                self.change_n += 1

        elif self.right_maneuver:
            if self.d_lat < -1.5:
                out_right = self.blink_state
                self.change_n = 0
            else:
                out_right = self.blink_state
                self.change_n += 1

        # Safety timeout: If 4 seconds elapse, the control system shuts down.
        if self.change_n >= 40:
            self.left_maneuver = False
            self.right_maneuver = False
            self.change_n = 0

        # -------------------------------------------------
        # LOGIC 3: Brake light and parking lights (do not interfere with turn signals)
        # -------------------------------------------------
        if v_current > self.threshold:
            self.is_moving = True
            self.stop_start_time = None
            
            # Dynamic deceleration logic
            if v_current >= self.v_prev:
                self.reference_v = v_current
                out_brake = False
            else:
                caida = self.reference_v - v_current
                out_brake = caida > self.brake_delta
        else:
            # Vehicle Stopped
            if self.is_moving:
                self.is_moving = False
                self.stop_start_time = now
                self.reference_v = 0.0

            if self.stop_start_time is None:
                out_brake = False
                # If you are not changing lanes, activate your hazard lights for safety.
                if not self.left_maneuver and not self.right_maneuver:
                    out_left = self.blink_state
                    out_right = self.blink_state
            else:
                elapsed = (now - self.stop_start_time).nanoseconds / 1e9
                if elapsed < self.stop_timeout:
                    out_brake = True
                    # It keeps the lane-change signals active if a maneuver was already in progress.
                else:
                    out_brake = False
                    # Permanent Parking Mode: If there is no lane crossing, it activates the hazard lights.
                    if not self.left_maneuver and not self.right_maneuver:
                        out_left = self.blink_state
                        out_right = self.blink_state

        # Save speeds
        self.v_prev = v_current

        # SINGLE PUBLICATION
        self.publish_all(out_brake, out_left, out_right)

    def publish_all(self, brake, left, right):
        m_stop = Bool(data=brake)
        m_left = Bool(data=left)
        m_right = Bool(data=right)
        
        self.pub_stop.publish(m_stop)
        self.pub_left.publish(m_left)
        self.pub_right.publish(m_right)

def main(args=None):
    rclpy.init(args=args)
    node = RobotLightsController()
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
