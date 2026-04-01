import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies
from scipy.spatial.transform import Rotation as R

class OptitrackTestNode(Node):
    def __init__(self):
        super().__init__('optitrack_test_node')

        # Subscribe to the rigid bodies topic
        self.create_subscription(RigidBodies, '/rigid_bodies', self.rigid_bodies_callback, 10)
        
        self.get_logger().info("OptiTrack Test Node started. Listening for rigid body '2' on '/rigid_bodies'...")

    def rigid_bodies_callback(self, msg):
        # Find the specific rigid body named '1'
        robot_body = next((rb for rb in msg.rigidbodies if rb.rigid_body_name == '2'), None)
        
        if robot_body is None:
            # Uncomment the next line if you want to know when the body is missing from a frame
            # self.get_logger().warning("Rigid body '1' not found in current mocap frame.", throttle_duration_sec=2.0)
            return

        pos = robot_body.pose.position
        ori = robot_body.pose.orientation

        # Apply your coordinate transformations
        r_mocap = R.from_quat([ori.x, ori.y, ori.z, ori.w])
        R_correction = R.from_euler('z', -90, degrees=True)
        r_ros = R_correction * r_mocap
        
        # Extract euler angles (converted to degrees for easier debugging)
        _, _, yaw_deg = r_ros.as_euler('xyz', degrees=True)

        # Log the parsed data to the terminal
        self.get_logger().info(
            f"Robot '2' | X: {pos.x:+.3f}, Y: {pos.y:+.3f}, Z: {pos.z:+.3f} | Yaw: {yaw_deg:+.2f}°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OptitrackTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()