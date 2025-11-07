#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies, Markers
from scipy.spatial.transform import Rotation as R
from mocap_listener import sabertooth as st
import atexit
from mocap_listener import PID as PID

REFRESH_RATE = 10 #hz


class ControllerNode(Node):
    def __init__(self):
        print("Starting GTG Controller node")
        super().__init__('controller_node')

        # Subscribe to mocap data
        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.rigid_bodies_listener_callback,
            REFRESH_RATE)
        
        self.subscription = self.create_subscription(
            Markers,
            '/markers',
            self.markers_listener_callback,
            REFRESH_RATE)
        
        self.timer = self.create_timer(1/REFRESH_RATE, self.controller_update)
        self.latest_rigidbodies_msg = None
        self.latest_markers_msg = None
    


    def rigid_bodies_listener_callback(self, msg: RigidBodies):
        self.latest_rigidbodies_msg = msg  # always store the newest message

    def markers_listener_callback(self, msg: Markers):
        self.latest_markers_msg = msg  # always store the newest message

        

    def controller_update(self):
        # self.get_logger().info("Updating controller")
        
        robot_body = None
        if self.latest_rigidbodies_msg != None:
            robot_body = next((rb for rb in self.latest_rigidbodies_msg.rigidbodies if rb.rigid_body_name == '1'), None)
        if robot_body is None:
            self.get_logger().info("Lost Body")
            return
        
        pos = robot_body.pose.position
        ori = robot_body.pose.orientation

        r = R.from_quat([ori.x, ori.y, ori.z, ori.w])
        _, _, yaw = r.as_euler('xyz', degrees=False)


        x_des = 0
        y_des = 0
        goal = None
        if self.latest_markers_msg != None:
            goal = next((pt for pt in self.latest_markers_msg.markers if pt.marker_index == 3), None)
        if goal is None:
            self.get_logger().warn("Lost Goal! Dumping data:")
            print(self.latest_markers_msg.markers)
        else:
            x_des = goal.translation.x
            y_des = goal.translation.y
        
        

        

        self.get_logger().info(
            f"X={pos.x:.3f}, Y={pos.y:.3f}, Dir={yaw:.3f}  |  Goal: {x_des:.3f}, {y_des:.3f}"
        )
        

        

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    motor = st.SaberToothMotorDriver(True,True)

    # motor.updateMotorSpeed(20,20)

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
