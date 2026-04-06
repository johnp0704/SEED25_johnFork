import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import time
import atexit

# Import your motor driver
from opti_waypoint_nav import sabertooth as st 

class SimpleObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('simple_obstacle_avoidance')
        self.get_logger().info("Starting Simple Obstacle Avoidance")

        # --- State Machine Variables ---
        self.state = 'FORWARD'
        self.state_start_time = time.time()
        
        # --- Tuning Parameters ---
        self.SAFE_DISTANCE = 0.5   # meters (Threshold to trigger avoidance)
        self.BACKUP_DURATION = 1.0 # seconds to drive backwards
        self.TURN_DURATION = 1.0   # seconds to spin in place
        
        self.FWD_SPEED = 15
        self.REV_SPEED = -15
        self.TURN_SPEED = 15

        # --- Initialize Motors ---
        try:
            self.motor = st.SaberToothMotorDriver(True, True)
            atexit.register(self.motor.all_motors_off)
        except Exception as e:
            self.get_logger().error(f"Motor error: {e}")

        # --- Initialize RealSense Depth Only ---
        self.pipeline = rs.pipeline()
        config = rs.config()
        # 640x480 is plenty of resolution for simple depth checking
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        # --- Control Loop (Runs at 10Hz) ---
        self.timer = self.create_timer(0.1, self.control_loop)

    def get_center_distance(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return 999.0 # Safe default if frame drops

        width = depth_frame.get_width()
        height = depth_frame.get_height()

        # Sample a 50x50 pixel box in the dead center of the image
        center_x, center_y = width // 2, height // 2
        distances = []
        
        for y in range(center_y - 25, center_y + 25):
            for x in range(center_x - 25, center_x + 25):
                dist = depth_frame.get_distance(x, y)
                if dist > 0.1: # Filter out 0.0 (camera blind spots/noise)
                    distances.append(dist)

        if not distances:
            return 999.0

        return float(np.mean(distances))

    def control_loop(self):
        distance = self.get_center_distance()
        current_time = time.time()
        elapsed_in_state = current_time - self.state_start_time

        if self.state == 'FORWARD':
            if distance < self.SAFE_DISTANCE:
                self.get_logger().warn(f"OBSTACLE DETECTED at {distance:.2f}m! Backing up...")
                self.state = 'BACKING_UP'
                self.state_start_time = current_time
                self.motor.updateMotorSpeed(self.REV_SPEED, self.REV_SPEED)
            else:
                self.get_logger().info(f"Path clear ({distance:.2f}m). Moving forward.")
                self.motor.updateMotorSpeed(self.FWD_SPEED, self.FWD_SPEED)

        elif self.state == 'BACKING_UP':
            if elapsed_in_state > self.BACKUP_DURATION:
                self.get_logger().warn("Turning to find a clear path...")
                self.state = 'TURNING'
                self.state_start_time = current_time
                # Spin in place (Left wheel reverse, Right wheel forward)
                self.motor.updateMotorSpeed(-self.TURN_SPEED, self.TURN_SPEED)

        elif self.state == 'TURNING':
            if elapsed_in_state > self.TURN_DURATION:
                self.get_logger().info("Turn complete. Resuming forward motion.")
                self.state = 'FORWARD'
                self.state_start_time = current_time
                self.motor.updateMotorSpeed(self.FWD_SPEED, self.FWD_SPEED)

    def destroy_node(self):
        self.motor.all_motors_off()
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()