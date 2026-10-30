#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import psutil
import subprocess
import os
import time
from datetime import datetime
import glob
from std_msgs.msg import Bool

class SystemHealthMonitor(Node):
    def __init__(self):
        super().__init__('system_health_monitor')
        
        # 1. Declare parameters with default values
        self.declare_parameter('cpu_temp_warning', 85.0)
        self.declare_parameter('cpu_temp_critical', 95.0)
        self.declare_parameter('gpu_temp_warning', 80.0)
        self.declare_parameter('gpu_temp_critical', 90.0)
        self.declare_parameter('disk_temp_warning', 55.0)
        self.declare_parameter('battery_warning', 25.0)
        self.declare_parameter('battery_critical', 15.0)
        self.declare_parameter('cpu_usage_warning', 90.0)
        self.declare_parameter('ram_usage_warning', 90.0)
        self.declare_parameter('update_period', 2.0)

        # Publishers
        self.cpu_temp_pub = self.create_publisher(DiagnosticStatus, '/status/cpu_temperature', 10)
        self.gpu_temp_pub = self.create_publisher(DiagnosticStatus, '/status/gpu_temperature', 10)
        self.battery_pub = self.create_publisher(DiagnosticStatus, '/status/battery_laptop', 10)
        self.ram_pub = self.create_publisher(DiagnosticStatus, '/status/ram', 10)
        self.cpu_usage_pub = self.create_publisher(DiagnosticStatus, '/status/cpu_usage', 10)
        self.gpu_usage_pub = self.create_publisher(DiagnosticStatus, '/status/gpu_usage', 10)
        self.disk_temp_pub = self.create_publisher(DiagnosticStatus, '/status/disk_temperature', 10)
        self.uptime_pub = self.create_publisher(DiagnosticStatus, '/status/uptime', 10)
        self.system_summary_pub = self.create_publisher(DiagnosticStatus, '/status/system_summary', 10)
        self.lid_pub = self.create_publisher(Bool, '/lid_closed', 10)

        # Current status and history
        self.current_state = {
            'cpu_temp': 0.0, 'gpu_temp': 0.0, 'battery_percent': 100.0,
            'battery_plugged': True, 'cpu_usage': 0.0, 'gpu_usage': 0.0,
            'ram_usage': 0.0, 'ram_used_gb': 0.0, 'ram_total_gb': 0.0,
            'disk_temp': 0.0, 'disk_usage': 0.0, 'disk_free_gb': 0.0,
            'uptime': 0.0, 'fan_rpm': 0
        }
        self.temp_history = {'cpu': [], 'gpu': [], 'max_history': 10}
        
        # Timers
        period = self.get_parameter('update_period').get_parameter_value().double_value
        self.timer = self.create_timer(period, self.publish_diagnostics)
        self.safety_timer = self.create_timer(5.0, self.safety_checks)
        
        self.gpu_type = self.detect_gpu_type()

    # --- Helper function to read system parameter limits. ---
    def get_limit(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    # --- Métodos de obtención de datos (Sin cambios en lógica interna) ---
    def detect_gpu_type(self):
        try:
            if subprocess.run(['which', 'nvidia-smi'], capture_output=True).returncode == 0: return "nvidia"
            if subprocess.run(['which', 'intel_gpu_top'], capture_output=True).returncode == 0: return "intel"
        except: pass
        return "unknown"

    def get_cpu_temperature(self):
        max_temp = 0.0
        for i in range(15):
            path = f'/sys/class/thermal/thermal_zone{i}/temp'
            if os.path.exists(path):
                with open(path, 'r') as f:
                    t = float(f.read().strip()) / 1000.0
                    if t > max_temp: max_temp = t
        return max_temp if max_temp > 0 else (40.0 + psutil.cpu_percent()*0.3)

    def get_gpu_temperature(self):
        try:
            if self.gpu_type == "nvidia":
                res = subprocess.run(['nvidia-smi', '--query-gpu=temperature.gpu', '--format=csv,noheader'], capture_output=True, text=True)
                return float(res.stdout.strip())
        except: pass
        return 0.0

    def get_battery_info(self):
        bat = psutil.sensors_battery()
        return {'percent': bat.percent, 'plugged': bat.power_plugged} if bat else {'percent': 100.0, 'plugged': True}

    def get_disk_temperature(self):
        files = glob.glob('/sys/class/nvme/nvme*/device/hwmon/hwmon*/temp*_input')
        for f_path in files:
            try:
                with open(f_path, 'r') as f: return float(f.read().strip()) / 1000.0
            except: continue
        return 45.0

    def get_system_stats(self):
        ram = psutil.virtual_memory()
        disk = psutil.disk_usage('/')
        return {
            'cpu_usage': psutil.cpu_percent(),
            'ram_usage': ram.percent,
            'ram_total_gb': ram.total / (1024**3),
            'ram_used_gb': ram.used / (1024**3),
            'disk_usage': disk.percent,
            'uptime': (time.time() - psutil.boot_time()) / 3600
        }

    def get_lid_state(self):
        paths = glob.glob('/proc/acpi/button/lid/*/state')
        if paths:
            with open(paths[0], 'r') as f: return "closed" in f.read().lower()
        return False

    def update_current_state(self):
        self.current_state['cpu_temp'] = self.get_cpu_temperature()
        self.current_state['gpu_temp'] = self.get_gpu_temperature()
        self.current_state['disk_temp'] = self.get_disk_temperature()
        battery = self.get_battery_info()
        self.current_state.update({'battery_percent': battery['percent'], 'battery_plugged': battery['plugged']})
        if self.gpu_type == "nvidia":
            try:
                res = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'], capture_output=True, text=True)
                self.current_state['gpu_usage'] = float(res.stdout.strip())
            except: pass
        self.current_state.update(self.get_system_stats())

    # --- Level Logic Using Parameters ---
    def get_temp_level(self, temp, component='cpu'):
        if component == 'cpu':
            warn, crit = self.get_limit('cpu_temp_warning'), self.get_limit('cpu_temp_critical')
        elif component == 'gpu':
            warn, crit = self.get_limit('gpu_temp_warning'), self.get_limit('gpu_temp_critical')
        else:
            warn = self.get_limit('disk_temp_warning')
            crit = warn + 10.0
        
        if temp >= crit: return DiagnosticStatus.ERROR
        if temp >= warn: return DiagnosticStatus.WARN
        return DiagnosticStatus.OK

    def get_battery_level(self, percent):
        if percent <= self.get_limit('battery_critical'): return DiagnosticStatus.ERROR
        if percent <= self.get_limit('battery_warning'): return DiagnosticStatus.WARN
        return DiagnosticStatus.OK

    def get_usage_level(self, usage, is_ram=True):
        limit = self.get_limit('ram_usage_warning') if is_ram else self.get_limit('cpu_usage_warning')
        return DiagnosticStatus.WARN if usage >= limit else DiagnosticStatus.OK

    def create_diagnostic_message(self, name, hardware_id, level, message, values=None):
        diag = DiagnosticStatus(name=name, hardware_id=hardware_id, level=level, message=message)
        if values:
            for k, v in values: diag.values.append(KeyValue(key=k, value=v))
        return diag

    def publish_diagnostics(self):
        self.update_current_state()
        
        self.cpu_temp_pub.publish(self.create_diagnostic_message(
            "CPU temperature", "laptop_cpu", self.get_temp_level(self.current_state['cpu_temp'], 'cpu'),
            f"{self.current_state['cpu_temp']:.1f}°C", [("temperature", f"{self.current_state['cpu_temp']:.1f}")]
        ))

        if self.current_state['gpu_temp'] > 0:
            self.gpu_temp_pub.publish(self.create_diagnostic_message(
                "GPU temperature", f"laptop_gpu_{self.gpu_type}", self.get_temp_level(self.current_state['gpu_temp'], 'gpu'),
                f"{self.current_state['gpu_temp']:.1f}°C", [("temperature", f"{self.current_state['gpu_temp']:.1f}")]
            ))

        self.battery_pub.publish(self.create_diagnostic_message(
            "Laptop Battery Percentage", "laptop_battery", self.get_battery_level(self.current_state['battery_percent']),
            f"{self.current_state['battery_percent']:.1f}%", [("percentage", f"{self.current_state['battery_percent']:.1f}")]
        ))

        self.ram_pub.publish(self.create_diagnostic_message(
            "RAM usage", "laptop_ram", self.get_usage_level(self.current_state['ram_usage'], True),
            f"{self.current_state['ram_usage']:.1f}%", [("usage_percent", f"{self.current_state['ram_usage']:.1f}")]
        ))

        self.cpu_usage_pub.publish(self.create_diagnostic_message(
            "CPU usage", "laptop_cpu_usage", self.get_usage_level(self.current_state['cpu_usage'], False),
            f"{self.current_state['cpu_usage']:.1f}%", [("usage_percent", f"{self.current_state['cpu_usage']:.1f}")]
        ))

        self.disk_temp_pub.publish(self.create_diagnostic_message(
            "Disk temperature", "laptop_disk", self.get_temp_level(self.current_state['disk_temp'], 'disk'),
            f"{self.current_state['disk_temp']:.1f}°C", [("temperature", f"{self.current_state['disk_temp']:.1f}")]
        ))

        # Lid State
        lid_msg = Bool(data=self.get_lid_state())
        self.lid_pub.publish(lid_msg)

    def safety_checks(self):
        pass

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SystemHealthMonitor()
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