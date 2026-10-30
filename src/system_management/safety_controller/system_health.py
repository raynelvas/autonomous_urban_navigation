#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from jtop import jtop, JtopException

class JetsonHealthMonitor(Node):
    def __init__(self):
        super().__init__('jetson_health_monitor')

        # 1. Declare parameters with default values
        self.declare_parameter('soc_warn', 75.0)
        self.declare_parameter('soc_crit', 85.0)
        self.declare_parameter('vin_warn', 4.85)
        self.declare_parameter('vin_crit', 4.75)
        self.declare_parameter('ram_warn', 70.0)
        self.declare_parameter('ram_crit', 90.0)
        self.declare_parameter('cpu_warn', 70.0)
        self.declare_parameter('cpu_crit', 85.0)
        self.declare_parameter('fan_warn', 60.0)
        self.declare_parameter('fan_crit', 80.0)
        self.declare_parameter('update_period', 1.0)

        # Publishers
        self.pubs = {
            'soc': self.create_publisher(DiagnosticStatus, '/status/soc_temperature', 10),
            'gpu_temp': self.create_publisher(DiagnosticStatus, '/status/gpu_temperature', 10),
            'cpu_temp': self.create_publisher(DiagnosticStatus, '/status/cpu_temperature', 10),
            'vin': self.create_publisher(DiagnosticStatus, '/status/input_voltage', 10),
            'pow': self.create_publisher(DiagnosticStatus, '/status/power_usage', 10),
            'gpu': self.create_publisher(DiagnosticStatus, '/status/gpu_usage', 10),
            'cpu': self.create_publisher(DiagnosticStatus, '/status/cpu_usage', 10),
            'ram': self.create_publisher(DiagnosticStatus, '/status/ram_usage', 10),
            'fan': self.create_publisher(DiagnosticStatus, '/status/fan_speed', 10)
        }

        self.gpu_history = []

        # jtop initialization
        self.jetson = jtop()
        try:
            self.jetson.start()
        except JtopException as e:
            self.get_logger().error(f"Could not start jtop: {e}")

        # Timer using the loaded parameter
        period = self.get_parameter('update_period').get_parameter_value().double_value
        self.timer = self.create_timer(period, self.update_and_publish)
        
        
    # --- Helper function to quickly retrieve parameters ---
    def get_limit(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def create_diag(self, name, level, message, values=None):
        diag = DiagnosticStatus()
        diag.name = name
        diag.level = bytes([int(level)])
        diag.message = message
        diag.hardware_id = "orin_nano"
        if values:
            for k, v in values.items():
                diag.values.append(KeyValue(key=str(k), value=str(v)))
        return diag
    
    def get_diag_level(self, value, warn_param, crit_param, reverse=False):
        # We read the current parameter values ​​each time
        warn = self.get_limit(warn_param)
        crit = self.get_limit(crit_param)
        
        if value == 0 and reverse: return 3
        if reverse:
            if value < crit: return 2
            if value < warn: return 1
        else:
            if value > crit: return 2
            if value > warn: return 1
        return 0

    def update_and_publish(self):
        if not self.jetson.ok(): return

        try:
            # -------- TEMPERATURE --------
            temps = self.jetson.temperature
            cpu_t = float(temps.get('cpu', {}).get('temp', 0.0))
            gpu_t = float(temps.get('gpu', {}).get('temp', 0.0))
            tj_t = float(temps.get('tj', {}).get('temp', 0.0))
            soc_t = max(cpu_t, gpu_t, tj_t)

            soc_l = self.get_diag_level(soc_t, 'soc_warn', 'soc_crit')
            gpu_tl = self.get_diag_level(gpu_t, 'soc_warn', 'soc_crit')
            cpu_tl = self.get_diag_level(cpu_t, 'soc_warn', 'soc_crit')

            self.pubs['gpu_temp'].publish(self.create_diag("GPU Temperature", gpu_tl, f"{gpu_t:.2f}°C", {"temp": gpu_t}))
            self.pubs['cpu_temp'].publish(self.create_diag("CPU Temperature", cpu_tl, f"{cpu_t:.2f}°C", {"temp": cpu_t}))
            self.pubs['soc'].publish(self.create_diag("SOC Temperature", soc_l, f"{soc_t:.2f}°C", {"temp": soc_t}))

            # -------- ENERGY --------
            p_info = self.jetson.power.get('tot', {})
            vin = float(p_info.get('volt', 0.0)) / 1000.0
            p_mw = float(p_info.get('avg', 0.0))
            curr = float(p_info.get('curr', 0.0)) / 1000.0

            vin_l = self.get_diag_level(vin, 'vin_warn', 'vin_crit', reverse=True)
            self.pubs['vin'].publish(self.create_diag("Input Voltage", vin_l, f"{vin:.2f} V", {"voltage": vin}))
            self.pubs['pow'].publish(self.create_diag("Power System", 0, f"{p_mw/1000.0:.2f} W | {curr:.2f} A", {"p_mw": p_mw, "curr": curr}))

            # -------- CPU / RAM / FAN --------
            # CPU
            idle = float(self.jetson.cpu.get('total', {}).get('idle', 100.0))
            cpu_u = 100.0 - idle
            cpu_l = self.get_diag_level(cpu_u, 'cpu_warn', 'cpu_crit')
            self.pubs['cpu'].publish(self.create_diag("CPU Usage", cpu_l, f"{cpu_u:.1f}%", {"usage": cpu_u}))
            
            # GPU
            gpu_info = self.jetson.gpu
            gpu_u = float(gpu_info.get('gpu', {}).get('status', {}).get('load', 0.0))
            gpu_l = self.get_diag_level(gpu_u, 'cpu_warn', 'cpu_crit')
            self.pubs['gpu'].publish( self.create_diag("GPU Usage", gpu_l, f"{gpu_u:.1f}%", {"usage": gpu_u}))

            # RAM
            ram = self.jetson.memory.get('RAM', {})
            ram_pct = (float(ram.get('used', 0)) / float(ram.get('tot', 1))) * 100
            ram_l = self.get_diag_level(ram_pct, 'ram_warn', 'ram_crit')
            self.pubs['ram'].publish(self.create_diag("RAM Usage", ram_l, f"{ram_pct:.1f}%", {"usage": ram_pct}))

            # FAN
            fan_data = getattr(self.jetson, 'fan', {})
            f_speed = 0.0
            for _, d in fan_data.items():
                f_speed = float(d.get('speed', [0])[0])
                break
            fan_l = self.get_diag_level(f_speed, 'fan_warn', 'fan_crit')
            self.pubs['fan'].publish(self.create_diag("Fan Speed", fan_l, f"{f_speed:.1f}%", {"speed": f_speed}))

        except Exception as e:
            self.get_logger().error(f"Loop error: {e}")

    def destroy_node(self):
        self.jetson.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JetsonHealthMonitor()
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