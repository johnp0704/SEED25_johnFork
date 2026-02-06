#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies
import tf.transformations as tf

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Subscribe to mocap data
        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.listener_callback,
            10)

        

    def listener_callback(self, msg: RigidBodies):
        target = next((rb for rb in msg.rigidbodies if rb.rigid_body_name == '1'), None)
        if target is None:
            return

        pos = target.pose.position
        ori = target.pose.orientation

        _, _, yaw = tf.euler_from_quaternion(ori)

        self.get_logger().info(
                f"X=({pos.x:.3f}, Y={pos.y:.3f}, Dir=({yaw:.3f}"  
            )

        

        

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
