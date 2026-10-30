#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import casadi as ca
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from custom_interfaces.msg import LaneModel

from diagnostic_msgs.msg import DiagnosticStatus

class LaneControllerNMPC(Node):

    def __init__(self):
        super().__init__('lane_controller_nmpc')

        # Declare and retrieve parameters from the YAML
        # If they don't exist in the YAML, they will take the default value
        self.N = self.declare_parameter('N', 10).value
        self.dt = self.declare_parameter('dt', 0.1).value
        self.v_ref = self.declare_parameter('v_ref', 1.5).value
        self.k_curve = self.declare_parameter('k_curve', 2.5).value

        self.Q_ey = self.declare_parameter('Q_ey', 10.0).value
        self.Q_epsi = self.declare_parameter('Q_epsi', 12.5).value
        self.R_accel = self.declare_parameter('R_accel', 1.0).value
        self.R_omega = self.declare_parameter('R_omega', 2.5).value
        self.S_accel = self.declare_parameter('S_accel', 2.0).value
        self.S_omega = self.declare_parameter('S_omega', 3.0).value
        self.W_v_follow = self.declare_parameter('W_v_follow', 5.0).value
        self.Q_terminal = self.declare_parameter('Q_terminal', 15.0).value

        # Physical limits
        self.declare_parameter('max_lin_accel', 0.05)
        self.declare_parameter('decel_vision', 1.0)
        self.declare_parameter('decel_system', 0.5)
        self.declare_parameter('decel_safe_stop', 0.1)
        self.declare_parameter('v_max', 1.7)
        self.declare_parameter('w_max', 0.6)
        self.declare_parameter('min_v_start', 0.1)
        self.declare_parameter('max_v_start', 0.2)

        # Subs/Pubs
        self.create_subscription(LaneModel, '/lane/model_filtered', self.lane_cb, 10)
        self.create_subscription(Odometry, '/odometry/local', self.odom_cb, 10)
        self.create_subscription(Bool, '/emergency', self.emergency_cb, 10)
        self.create_subscription(Bool, '/safe_stop', self.safe_stop_cb, 10)
        self.create_subscription(Float32, '/alpha/vision', self.vision_alpha_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/active/vision', self.active_vision_cb, 10)
        self.create_subscription(Bool, '/manual', self.manual_mode_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_manual', self.manual_twist_cb, 10)

        # 2. Global State Subscription
        self.create_subscription(
            DiagnosticStatus, 
            '/global_status', 
            self.global_status_cb, 
            10)

        self.is_manual = False
        self.manual_twist = Twist()
        
        # Variable to store the state (0=OK, 1=WARN, 2=ERROR según estándar)
        self.global_level = 0

        self.alpha_vision = 1.0
        self.active_vision = False

        self.alpha_system = 1.0  # What the global system (security) says

        # State [ey, epsi, v]
        self.state = np.zeros(3)
        self.kappa = 0.0
        self.emergency = True
        self.safe_stop = True

        # Warm start memory
        self.u_prev = np.zeros(2)

        self.current_alpha = 1.0

        # Variables for emergency stop by artificial vision
        self.count_plus = 0
        self.count_minus = 0
        self.emergency_vision = False

        self.setup_nmpc()
        self.create_timer(0.033, self.control_loop)
    
    # ============ LINEAR SPEED CONTROL BY OBJECT DETECTION =====================
    def vision_alpha_cb(self, msg):
        self.alpha_vision = msg.data

        if self.alpha_vision < 0.05:
            self.count_plus += 1
            self.count_minus = 0
        else:
            self.count_minus += 1
            self.count_plus = 0

        if self.count_plus >= 5:
            self.emergency_vision = True
        elif self.count_minus >= 5:
            self.emergency_vision = False

    # ============================================================
    # SETUP NMPC
    # ============================================================
    def setup_nmpc(self):
        nx = 3
        nu = 2

        x = ca.SX.sym('x', nx)
        u = ca.SX.sym('u', nu)
        kappa_p = ca.SX.sym('kappa')

        ey_dot = x[2] * ca.sin(x[1])
        denom = 1 + kappa_p * x[0]
        denom_safe = denom + 1e-6
        epsi_dot = -u[1] - (x[2] * kappa_p * ca.cos(x[1])) / denom_safe
        v_dot = u[0]

        f = ca.Function('f', [x, u, kappa_p], [ca.vertcat(ey_dot, epsi_dot, v_dot)])

        U = ca.SX.sym('U', nu, self.N)
        X = ca.SX.sym('X', nx, self.N + 1)

        # P = [ey0, epsi0, v0, kappa, u_prev_accel, u_prev_omega, speed_lineal]
        P = ca.SX.sym('P', nx + 1 + nu + 1)

        cost = 0
        X[:, 0] = P[0:3]

        for k in range(self.N):
            st = X[:, k]
            con = U[:, k]

            # 1. Lateral and angular error
            cost += self.Q_ey * st[0]**2
            cost += self.Q_epsi * st[1]**2

            # 2. Adaptive speed
            v_ref_dinamica = self.v_ref * P[6] 
            v_target = v_ref_dinamica / (1.0 + self.k_curve * ca.fabs(P[3]))
            cost += self.W_v_follow * (st[2] - v_target)**2

            # 3. Control magnitude
            cost += self.R_accel * con[0]**2
            cost += self.R_omega * con[1]**2

            # 4. Penalización de jerk
            if k == 0:
                delta_u = con - P[4:6]
            else:
                delta_u = con - U[:, k-1]

            cost += self.S_accel * delta_u[0]**2
            cost += self.S_omega * delta_u[1]**2

            # 5. RK4 Integration
            k1 = f(st, con, P[3])
            k2 = f(st + self.dt/2*k1, con, P[3])
            k3 = f(st + self.dt/2*k2, con, P[3])
            k4 = f(st + self.dt*k3, con, P[3])

            X[:, k+1] = st + self.dt/6*(k1 + 2*k2 + 2*k3 + k4)

        # Terminal Cost
        st_terminal = X[:, self.N]
        cost += self.Q_terminal * (st_terminal[0]**2 + st_terminal[1]**2)

        variables = ca.reshape(U, -1, 1)
        nlp = {'x': variables, 'f': cost, 'g': ca.vertcat(), 'p': P}

        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 50
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

        # We save last applied control
        self.u_prev = np.zeros(2)

    # ============================================================
    # CONTROL LOOP
    # ============================================================
    def control_loop(self):
        # 1. Priority: Manual Control
        if self.is_manual:
            # Dejamos pasar la señal del joystick directamente
            self.publish_cmd(self.manual_twist.linear.x, self.manual_twist.angular.z)
            return

        # 2. Emergency
        if self.emergency or not self.active_vision or self.emergency_vision:
            self.publish_cmd(0.0, 0.0)
            return
        
        # 3. SAFE STOP: We force alpha_vision to 0 so that the NMPC brakes smoothly
        self.current_alpha = min(self.alpha_vision, self.alpha_system)
        if self.safe_stop:
            self.current_alpha = 0.0

        # Parameters of the solver
        p = np.concatenate([
            self.state,
            [self.kappa],
            self.u_prev,
            [self.current_alpha] #for linear speed
        ])

        # Warm start simple
        u0 = np.tile(self.u_prev, self.N)

        # Obtain parameter limits
        d_vis = self.get_parameter('decel_vision').value
        d_sys = self.get_parameter('decel_system').value
        d_safe = self.get_parameter('decel_safe_stop').value
        accel_max = self.get_parameter('max_lin_accel').value
        w_max = self.get_parameter('w_max').value

        decel_active = d_sys

        if self.safe_stop:
            decel_active = d_safe

        if abs(self.alpha_vision-self.current_alpha) < 0.1:
            decel_active = d_vis
        elif abs(self.alpha_system-self.current_alpha) < 0.05:
            decel_active = d_sys    

        lbx = np.array([-decel_active, -w_max] * self.N)
        ubx = np.array([ accel_max,  w_max] * self.N)

        try:
            res = self.solver(x0=u0, p=p, lbx=lbx, ubx=ubx)
            u_opt = res['x'].full().flatten()

            accel_cmd = float(u_opt[0])
            omega_cmd = float(u_opt[1])

            # We save the applied control
            self.u_prev = np.array([accel_cmd, omega_cmd])

        except:
            accel_cmd = 0.0
            omega_cmd = self.u_prev[1]  # maintain previous rotation

        v_final = self.state[2] + accel_cmd * self.dt

        self.publish_cmd(v_final, omega_cmd)
    # ============================================================

    def lane_cb(self, msg):
        self.state[0] = msg.d_lat
        self.state[1] = msg.yaw
        self.kappa = msg.curvature

    def odom_cb(self, msg):
        self.state[2] = msg.twist.twist.linear.x

    def emergency_cb(self, msg):
        self.emergency = msg.data
    
    def safe_stop_cb(self, msg):
        self.safe_stop = msg.data

    def active_vision_cb(self, msg):
        self.active_vision = msg.data
    
    def global_status_cb(self, msg):
        # msg.level es un byte: 0=OK, 1=WARN, 2=ERROR
        self.global_level = int.from_bytes(msg.level, byteorder='little', signed=False)
        if self.global_level == 1: # WARNING
            self.alpha_system  = 0.5            
        elif self.global_level == 2: # ERROR
            self.alpha_system  = 0.0
        else: 
            self.alpha_system = 1.0
    
    def manual_mode_cb(self, msg):
        previous_manual = self.is_manual
        self.is_manual = msg.data
        
        # If we have just released manual control (switched from True to False)
        if previous_manual and not self.is_manual:
            # We clear the solver's memory for a soft start
            self.u_prev = np.array([0.0, self.manual_twist.angular.z])

    def manual_twist_cb(self, msg):
        self.manual_twist = msg

    def publish_cmd(self, v, w):
        if not rclpy.ok(): 
            return
    
        v_min = self.get_parameter('min_v_start').value
        v_max = self.get_parameter('v_max').value
        v_threshold = self.get_parameter('max_v_start').value

        if not self.is_manual:
            if v <= v_min and self.current_alpha == 0.0:
                v = 0.0
                w = 0.0
            elif v <= v_threshold:
                scale = max(0.3, v / v_threshold)
                w *= scale

            if 0.001 < v < v_min and self.current_alpha != 0.0:
                v = v_min

        msg = Twist()
        msg.linear.x = v 
        msg.angular.z = float(w)

        self.pub_cmd.publish(msg)

# ============================================================
    def destroy_node(self):
        self.alpha_vision = 0.0
        self.alpha_system = 0.0  
        self.current_alpha = 0.0
        self.publish_cmd(0.0, 0.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LaneControllerNMPC()
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