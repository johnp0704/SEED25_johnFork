import sys
import serial
import serial.tools.list_ports
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# ============================================================
#  CONFIG
# ============================================================
BAUD_RATE = 115200
TIMEOUT   = 500      # seconds to wait for a response

RETURN_CODES = {
    "0": "OK",
    "1": "Error: move unsafe (tool stuck or not homed)",
    "2": "Error: reserved (2)",
    "3": "Error: reserved (3)",
    "4": "Error: reserved (4)",
    "5": "Error: reserved (5)",
    "6": "Error: reserved (6)",
    "7": "Error: reserved (7)",
    "8": "Error: reserved (8)",
    "9": "Error: reserved (9)",
}

VALID_COMMANDS = {"home", "drill"}
WINDOWS_IGNORED_PORTS = {"COM5", "COM6"}
IS_WINDOWS = sys.platform.startswith("win")

class AugerControllerNode(Node):
    def __init__(self):
        super().__init__('auger_controller_node')

        # Declare a ROS parameter for the serial port so it can be set in a launch file
        self.declare_parameter('serial_port', '')
        
        # State tracking
        self.is_tool_active = False

        # Publishers and Subscribers
        self.motor_disable_pub = self.create_publisher(Bool, '/motor_disable', 10)
        self.create_subscription(String, '/auger/activate', self.activation_callback, 10)

        # Initialize Serial Connection
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        if not port:
            port = self.find_port()

        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT)
            self.get_logger().info(f"Successfully connected to ESP32 on port: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port {port}: {e}")
            sys.exit(1)

    def find_port(self):
        """Automatically finds a port without blocking for user input."""
        ports = list(serial.tools.list_ports.comports())
        if IS_WINDOWS:
            ports = [p for p in ports if p.device.upper() not in WINDOWS_IGNORED_PORTS]
        
        if not ports:
            self.get_logger().error("No serial ports found. Cannot start Auger Node.")
            sys.exit(1)

        # Auto-select the first available port
        selected_port = ports[0].device
        if len(ports) > 1:
            self.get_logger().warning(f"Multiple ports found. Auto-selecting {selected_port}. Use 'serial_port' ROS parameter to specify exact port.")
            for i, p in enumerate(ports):
                self.get_logger().info(f"  Available: {p.device} — {p.description}")
        
        return selected_port

    def activation_callback(self, msg):
        """Triggered when a message is received on /auger/activate."""
        command = msg.data.strip().lower()

        if command not in VALID_COMMANDS:
            self.get_logger().warning(f"Ignored invalid command '{command}'. Valid: {VALID_COMMANDS}")
            return

        if self.is_tool_active:
            self.get_logger().warning("Auger is already currently active. Ignoring new command.")
            return

        # Start the sequence in a background thread. 
        threading.Thread(target=self.execute_tool_cycle, args=(command,), daemon=True).start()

    def execute_tool_cycle(self, command):
        """Runs the actual ESP32 communication and manages motor safety."""
        self.is_tool_active = True
        
        # 1. Shut down all motors
        self.get_logger().info("Disabling drivetrain motors for tool operation...")
        disable_msg = Bool()
        disable_msg.data = True
        self.motor_disable_pub.publish(disable_msg)

        # 2. Send command to ESP32
        self.get_logger().info(f"Sending '{command}' command to ESP32...")
        try:
            self.ser.reset_input_buffer()
            self.ser.write((command + "\n").encode())
            
            # This line blocks until the ESP32 responds or the 500s timeout is reached
            raw_response = self.ser.readline().decode().strip()
            
            code_str = RETURN_CODES.get(raw_response, f"Unknown response: '{raw_response}'")
            self.get_logger().info(f"ESP32 Response: {raw_response} — {code_str}")

        except Exception as e:
            self.get_logger().error(f"Serial communication failed: {e}")

        finally:
            # 3. Resume motors safely
            self.get_logger().info("Tool operation complete. Re-enabling drivetrain motors.")
            disable_msg.data = False
            self.motor_disable_pub.publish(disable_msg)
            
            self.is_tool_active = False

    def destroy_node(self):
        """Override the default destroy_node to catch Ctrl+C safely."""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.get_logger().info("Node shutting down. Sending 'stop' command to ESP32...")
            try:
                # Force a stop command to the tool
                self.ser.write(b"stop\n")
                self.ser.flush()  # Ensure the bytes leave the buffer before closing
            except Exception as e:
                self.get_logger().error(f"Failed to send stop command during shutdown: {e}")
            finally:
                self.ser.close()
                self.get_logger().info("Serial port closed.")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AugerControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # We moved the serial closing logic into node.destroy_node() to keep it object-oriented
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()