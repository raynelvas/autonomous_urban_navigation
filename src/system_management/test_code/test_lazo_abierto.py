#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv
import time
import math

class IdentificarMotor(Node):
    def __init__(self):
        super().__init__('identificar_motor')

        # ========= CONFIGURACIÓN EXPERIMENTO =========
        self.total_time = 60.0        # Duración total [s]
        self.sample_time = 0.1        # Tiempo entre comandos [s]
        self.save_path = '/home/raynel//autonomous_navigation/ESP32/test_codes/Matlab/identificacion_motor.csv'  # Archivo de salida

        # Secuencia de señales (puedes modificar)
        # Escalones de amplitud variable simétrica
        self.sequence = [0, 10, -10, 30, -30, 50, -50, 63, -63, 0]

        # Cada valor dura X segundos
        self.duration_per_level = 5.0  # segundos por nivel

        # =============================================
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom/unfiltered', self.odom_callback, 10)

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.last_cmd_time = self.start_time
        self.current_level_index = 0
        self.cmd_sent = 0

        # Para almacenamiento
        self.data = []  # [t_cmd, u_right, u_left, t_odom, w_right, w_left]

        # Timers
        self.timer_cmd = self.create_timer(self.sample_time, self.send_command)
        self.timer_stop = self.create_timer(self.total_time, self.stop_experiment)

        self.get_logger().info("Nodo de identificación iniciado.")

    def send_command(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time

        # Cambiar de nivel cada 'duration_per_level'
        if elapsed > (self.current_level_index + 1) * self.duration_per_level and \
           self.current_level_index < len(self.sequence) - 1:
            self.current_level_index += 1

        u = self.sequence[self.current_level_index]

        msg = Twist()
        msg.linear.x = float(u)      # Motor derecho
        msg.angular.z = float(u)     # Motor izquierdo
        self.cmd_pub.publish(msg)

        self.cmd_sent = u
        t_cmd = time.time()
        self.data.append([t_cmd, u, u, None, None, None])

    def odom_callback(self, msg):
        t_odom = time.time()
        w_right = msg.twist.twist.linear.x
        w_left = msg.twist.twist.angular.z

        # Añadir última fila (si existe) con odometría
        if len(self.data) > 0:
            self.data[-1][3] = t_odom
            self.data[-1][4] = w_right
            self.data[-1][5] = w_left

    def stop_experiment(self):
        self.get_logger().info("Finalizando experimento... Deteniendo motores.")
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.cmd_pub.publish(stop)

        # Guardar datos
        with open(self.save_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t_cmd', 'u_right', 'u_left', 't_odom', 'w_right', 'w_left'])
            writer.writerows(self.data)

        self.get_logger().info(f"Datos guardados en {self.save_path}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = IdentificarMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_experiment()

if __name__ == '__main__':
    main()
