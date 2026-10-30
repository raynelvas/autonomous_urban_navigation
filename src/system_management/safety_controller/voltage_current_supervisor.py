import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import time

class VoltageCurrentSupervisor(Node):
    def __init__(self):
        super().__init__('supervisor_voltage')
        self.declare_parameter('battery_12v_min', 11.0)
        self.declare_parameter('battery_12v_max', 13.0)
        self.declare_parameter('battery_percentage_min', 20.0)
        self.declare_parameter('battery_percentage_limit', 30.0)

        self.battery_12v_min = self.get_parameter('battery_12v_min').get_parameter_value().double_value
        self.battery_12v_max = self.get_parameter('battery_12v_max').get_parameter_value().double_value
        self.battery_percentage_min = self.get_parameter('battery_percentage_min').get_parameter_value().double_value
        self.battery_percentage_limit = self.get_parameter('battery_percentage_limit').get_parameter_value().double_value

        # Dedicated subscriber for battery data
        self.battery_sub = self.create_subscription(
            Float32MultiArray,
            '/battery_array',
            self.battery_callback,
            10
        )
        
        # Single publisher for 12V battery status
        self.battery_12v_pub = self.create_publisher(DiagnosticStatus, 'status/battery_12v', 10)
        
        # Single publisher for a relay controlling both motors
        self.motors_relay_pub = self.create_publisher(Bool, 'start/motors1', 10)
        
        # State variables
        self.current_voltage_12v = 0.0
        
        self.last_battery_time = None
        self.TIMEOUT_DURATION = 5.0  # seconds without data before considering it an error

        self.battery_connected = False
        
        self.battery_msg_count = 0
        
        # Timer for immediate publication at the start
        self.initial_publish_timer = self.create_timer(1.0, self.initial_status_publish)
        self.initial_publications_done = False
        
        # Timer for periodic timeout verification
        self.timeout_timer = self.create_timer(2.0, self.check_timeouts)

    def initial_status_publish(self):
        """Publishes initial states immediately upon node startup."""
        if not self.initial_publications_done:
            self.publish_initial_battery_status()
            
            self.initial_publications_done = True
            self.initial_publish_timer.cancel()

    def publish_initial_battery_status(self):
        """Publishes initial battery status (WAITING FOR DATA)"""
        battery_status = DiagnosticStatus()
        battery_status.name = "Battery voltage"
        battery_status.level = DiagnosticStatus.ERROR
        battery_status.message = "Awaiting initial data"
        
        battery_status.values = [
            self.create_key_value("voltage", "0.00"),
            self.create_key_value("percentage", "0.0"),
            self.create_key_value("units", "V"),
            self.create_key_value("status", "INICIALIZANDO"),
            self.create_key_value("message_count", "0"),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        
        self.battery_12v_pub.publish(battery_status)

    def battery_callback(self, msg):
        """Callback for battery data with timestamp"""
        current_time = time.time()
        self.last_battery_time = current_time
        self.battery_msg_count += 1
        
        if len(msg.data) >= 1:
            self.current_voltage_12v = msg.data[0]
            
            # Process only if it is the first time or if it recovered from a timeout.
            if not self.battery_connected:
                self.battery_connected = True
            
            self.publish_battery_status()
            self.check_emergency_shutdown()

    def check_timeouts(self):
        """Check for a timeout on the battery topic and publish an error status."""
        current_time = time.time()
        
        # Check battery timeout
        battery_timed_out = False
        if self.last_battery_time is not None:
            battery_timeout = current_time - self.last_battery_time > self.TIMEOUT_DURATION
            if battery_timeout and self.battery_connected:        
                self.battery_connected = False
                battery_timed_out = True
        elif self.last_battery_time is None and self.initial_publications_done:
            # If the initial time has already passed and you never received data
            if not hasattr(self, '_battery_never_received_logged') or not self._battery_never_received_logged:
                self._battery_never_received_logged = True
        
        # Post timeout error status
        if battery_timed_out:
            self.publish_battery_timeout_status()

    def publish_battery_timeout_status(self):
        """Reports battery timeout error status."""
        battery_status = DiagnosticStatus()
        battery_status.name = "Voltaje de la bateria"
        battery_status.level = DiagnosticStatus.ERROR
        battery_status.message = f"TIMEOUT: No data for {self.TIMEOUT_DURATION}s"
        
        battery_status.values = [
            self.create_key_value("voltage", "N/A"),
            self.create_key_value("percentage", "N/A"),
            self.create_key_value("units", "V"),
            self.create_key_value("status", "TIMEOUT"),
            self.create_key_value("last_message_count", str(self.battery_msg_count)),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        
        self.battery_12v_pub.publish(battery_status)

    def publish_battery_status(self):
        """Publishes normal battery status (only if connected)"""
        if not self.battery_connected:
            return
            
        # Calculate 12V battery percentage
        battery_percentage = max(0, min(100, 
            (self.current_voltage_12v - self.battery_12v_min) / 
            (self.battery_12v_max  - self.battery_12v_min) * 100
        ))
        
        battery_status = DiagnosticStatus()
        battery_status.name = "Battery voltage"
        
        if battery_percentage < self.battery_percentage_min:
            battery_status.level = DiagnosticStatus.ERROR
            battery_status.message = f"CRITICAL: {self.current_voltage_12v:.2f}V ({battery_percentage:.1f}%)"
        elif battery_percentage < self.battery_percentage_limit:
            battery_status.level = DiagnosticStatus.WARN
            battery_status.message = f"LOW: {self.current_voltage_12v:.2f}V ({battery_percentage:.1f}%)"
        else:
            battery_status.level = DiagnosticStatus.OK
            battery_status.message = f"OPTIMUM: {self.current_voltage_12v:.2f}V ({battery_percentage:.1f}%)"
        
        battery_status.values = [
            self.create_key_value("voltage", f"{self.current_voltage_12v:.2f}"),
            self.create_key_value("percentage", f"{battery_percentage:.1f}"),
            self.create_key_value("units", "V"),
            self.create_key_value("status", "ACTIVO"),
            self.create_key_value("message_count", str(self.battery_msg_count)),
            self.create_key_value("timestamp", f"{time.time():.2f}")
        ]
        
        self.battery_12v_pub.publish(battery_status)

    def check_emergency_shutdown(self):
        """Emergency voltage verification (only if valid data is available)"""
        if not self.battery_connected:
            return
        
        emergency = Bool()
        emergency.data = self.current_voltage_12v < self.battery_12v_min
        
        self.motors_relay_pub.publish(emergency)

    def create_key_value(self, key: str, value: str) -> KeyValue:
        """Helper function to create KeyValue objects"""
        kv = KeyValue()
        kv.key = key
        kv.value = value
        return kv

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoltageCurrentSupervisor()
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