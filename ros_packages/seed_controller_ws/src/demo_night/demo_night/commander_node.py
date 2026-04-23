import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
import time

class CommanderNode(Node):
    def __init__(self):
        super().__init__('commander_node')

        # --- State Variables ---
        self.current_mode = "IDLE"  # IDLE, REHOME, OPTICAL, TRAJECTORY
        self.gtg_active = False
        
        # --- Command Buffers ---
        self.cmd_gtg = [0.0, 0.0]
        self.cmd_rehome = [0.0, 0.0]
        self.cmd_optical = [0.0, 0.0]
        self.cmd_trajectory = [0.0, 0.0]
        
        # --- Timestamps for Timeout Safety ---
        self.last_gtg_time = 0
        self.last_state_time = 0

        # --- Publishers ---
        # This is the ONLY node that publishes to the actual motor driver
        self.motor_pub = self.create_publisher(Float32MultiArray, '/commander/wheel_cmd', 10)

        # --- Subscriptions ---
        # GUI State changes
        self.create_subscription(String, '/gui/system_state', self.gui_state_callback, 10)
        
        # Sub-controller command streams
        self.create_subscription(Float32MultiArray, '/vision/gtg_cmd', self.gtg_cmd_callback, 10)
        self.create_subscription(Float32MultiArray, '/vision/rehome_cmd', self.rehome_cmd_callback, 10)
        self.create_subscription(Float32MultiArray, '/vision/optical_cmd', self.optical_cmd_callback, 10)
        self.create_subscription(Float32MultiArray, '/nav/trajectory_cmd', self.trajectory_cmd_callback, 10)

        # Master control loop running at 20Hz
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Commander Node initialized. Standing by in IDLE mode.")

    def gui_state_callback(self, msg):
        self.current_mode = msg.data
        self.get_logger().info(f"GUI State changed to: {self.current_mode}")

    def gtg_cmd_callback(self, msg):
        self.cmd_gtg = [msg.data[0], msg.data[1]]
        self.last_gtg_time = self.get_clock().now().nanoseconds
        self.gtg_active = True

    def rehome_cmd_callback(self, msg):
        self.cmd_rehome = [msg.data[0], msg.data[1]]
        self.last_state_time = self.get_clock().now().nanoseconds

    def optical_cmd_callback(self, msg):
        self.cmd_optical = [msg.data[0], msg.data[1]]
        self.last_state_time = self.get_clock().now().nanoseconds

    def trajectory_cmd_callback(self, msg):
        self.cmd_trajectory = [msg.data[0], msg.data[1]]
        self.last_state_time = self.get_clock().now().nanoseconds

    def control_loop(self):
        now_ns = self.get_clock().now().nanoseconds
        final_cmd = [0.0, 0.0]

        # 1. Check GTG Priority (Timeout after 0.5 seconds of no detection)
        if (now_ns - self.last_gtg_time) < 5e8:
            final_cmd = self.cmd_gtg
        else:
            self.gtg_active = False
            
            # 2. Safety Timeout: If sub-controllers crash, stop the robot
            if (now_ns - self.last_state_time) > 1e9 and self.current_mode != "IDLE":
                self.get_logger().warn("Active controller timed out! Stopping motors.", throttle_duration_sec=2.0)
                final_cmd = [0.0, 0.0]
            
            # 3. Route based on GUI state
            elif self.current_mode == "REHOME":
                final_cmd = self.cmd_rehome
            elif self.current_mode == "OPTICAL":
                final_cmd = self.cmd_optical
            elif self.current_mode == "TRAJECTORY":
                final_cmd = self.cmd_trajectory
            else:
                final_cmd = [0.0, 0.0]  # IDLE

        # Publish final command to the motors
        msg = Float32MultiArray()
        msg.data = [float(final_cmd[0]), float(final_cmd[1])]
        self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()