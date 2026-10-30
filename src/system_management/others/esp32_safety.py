#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from diagnostic_msgs.msg import DiagnosticStatus
import serial
import threading

class SimpleSafetyNode(Node):
    def __init__(self):
        super().__init__('safety_unifier_simple')

        # --- Serial Configuration ---
        self.port = '/dev/ttyESP32Safety'
        self.ser = None
        self.connect_serial()
        
        # --- Internal State ---
        self.rele_bits = [False] * 8 
        self.last_voltage = 0.0
        self.connection_error = "Starting up..."

        # --- Subscribers ---
        self.create_subscription(Bool, '/light/stop',  lambda m: self.set_bit(0, m.data), 10)
        self.create_subscription(Bool, '/light/left',  lambda m: self.set_bit(1, m.data), 10)
        self.create_subscription(Bool, '/light/right', lambda m: self.set_bit(2, m.data), 10)
        self.create_subscription(Bool, '/light/safety',lambda m: self.set_bit(3, m.data), 10)
        self.create_subscription(Bool, '/start/motors', lambda m: self.set_bit(4, m.data), 10)

        # --- Publishers ---
        self.diag_pub = self.create_publisher(DiagnosticStatus, 'status/esp32_safety', 10)
        self.batt_pub = self.create_publisher(Float32MultiArray, 'battery_array', 10)

        # Timer
        self.create_timer(0.1, self.control_loop)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.05)
            self.connection_error = ""
        except Exception as e:
            self.connection_error = str(e)

    def set_bit(self, index, value):
        self.rele_bits[index] = value

    def control_loop(self):
        diag = DiagnosticStatus()
        diag.name = "ESP32 Safety"
        diag.hardware_id = "ESP32_Robot"

        # 1. Attempt to reconnect if the port was lost
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
            diag.level = DiagnosticStatus.ERROR
            diag.message = f"Desconectado"
            self.diag_pub.publish(diag)
            return

        # 2. Send bits to ESP32
        try:
            byte_val = 0
            for i, bit in enumerate(self.rele_bits):
                if bit:
                    byte_val |= (1 << i)
            
            cmd_string = f"CMD,{byte_val}\n"
            self.ser.write(cmd_string.encode('utf-8'))
            
        except Exception as e:
            self.ser = None 

        # 3. Read Data (Format: DATA,0.00,)
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                # We separate by comma: ['DATA', '0.00', '']
                parts = line.split(',')
                if len(parts) >= 2 and parts[0] == "DATA":
                    self.last_voltage = float(parts[1])
                    
                    batt_msg = Float32MultiArray()
                    batt_msg.data = [self.last_voltage]
                    self.batt_pub.publish(batt_msg)
            
            diag.level = DiagnosticStatus.OK
            diag.message = f"Voltaje: {self.last_voltage}V"
        except Exception as e:
            diag.level = DiagnosticStatus.WARN
            diag.message = f"Read error"

        self.diag_pub.publish(diag)

def main():
    rclpy.init()
    node = SimpleSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser: node.ser.close()
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()