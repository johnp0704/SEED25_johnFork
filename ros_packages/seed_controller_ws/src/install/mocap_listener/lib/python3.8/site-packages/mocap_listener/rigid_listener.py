#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies
import sabertooth as st
import time


class RigidBodyListener(Node):
    def __init__(self):
        super().__init__('rigid_body_listener')
        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.listener_callback,
            10)
        self.latest_msg = None

        # Create a timer to process data at 10 Hz (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.process_latest_msg)
	#self.timer = self.create_timer(1, self.process_latest_msg)

    def listener_callback(self, msg: RigidBodies):
        self.latest_msg = msg  # always store the newest message

    def process_latest_msg(self):
        if self.latest_msg is None:
            return
        msg = self.latest_msg
        self.get_logger().info(f"Frame: {msg.frame_number}")
        for rb in msg.rigidbodies:
            pos = rb.pose.position
            ori = rb.pose.orientation
            self.get_logger().info(
                f"RigidBody '{rb.rigid_body_name}': "
                f"pos=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) "
                f"ori=({ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f})"
            )

def main(args=None):
    rclpy.init(args=args)
    node = RigidBodyListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting!")
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
