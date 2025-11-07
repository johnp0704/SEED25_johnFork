#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies
from geometry_msgs.msg import Twist  # or whatever you plan to command

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Subscribe to mocap data
        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.listener_callback,
            10)

        # Publisher for velocity/commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def listener_callback(self, msg: RigidBodies):
        # For example: track rigid body '1'
        target = next((rb for rb in msg.rigidbodies if rb.rigid_body_name == '1'), None)
        if target is None:
            return

        pos = target.pose.position
        ori = target.pose.orientation

        # Example simple controller logic
        cmd = Twist()
        cmd.linear.x = -pos.x * 0.5
        cmd.linear.y = -pos.y * 0.5
        cmd.angular.z = -ori.z * 0.5

        self.publisher.publish(cmd)
        self.get_logger().info(f"Published control: x={cmd.linear.x:.3f}, y={cmd.linear.y:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
