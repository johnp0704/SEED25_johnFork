#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies

class RigidBodyListener(Node):
    def __init__(self):
        super().__init__('rigid_body_listener')

        # Subscribe to /rigid_bodies
        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: RigidBodies):
        self.get_logger().info(f"Frame: {msg.frame_number}")

        for rb in msg.rigidbodies:
            name = rb.rigid_body_name
            pos = rb.pose.position
            ori = rb.pose.orientation

            self.get_logger().info(
                f"RigidBody '{name}':\n"
                f"  Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}\n"
                f"  Orientation: x={ori.x:.3f}, y={ori.y:.3f}, z={ori.z:.3f}, w={ori.w:.3f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = RigidBodyListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
