#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies
from scipy.spatial.transform import Rotation as R
from mocap_listener import sabertooth as st
import atexit



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

        r = R.from_quat([ori.x, ori.y, ori.z, ori.w])
        _, _, yaw = r.as_euler('xyz', degrees=False)

        self.get_logger().info(
            f"X={pos.x:.3f}, Y={pos.y:.3f}, Dir={yaw:.3f}"
        )
        

        

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    motor = st.SaberToothMotorDriver(True,True)

    atexit.register(motor.all_motors_off)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting")
        
        motor.all_motors_off()


    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
