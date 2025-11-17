#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies, Markers
from scipy.spatial.transform import Rotation as R
from mocap_listener import sabertooth as st
import atexit
from mocap_listener import PID as PID
import numpy as np
import matplotlib.pyplot as plt



GOAL_MARKER_INDEX = 5
MAX_ACUTUATOR_INPUT = 30
S_MAX = (MAX_ACUTUATOR_INPUT-10) * 0.6
GOAL_THRESH = 0.3 # m
ANGLE_THRESH = np.deg2rad(10)

REFRESH_RATE = 10 #hz
R_wheel = .08 #cm
L = .178 #cm
K_e = 30
K_theta = -K_e * 20


class ControllerNode(Node):
    def __init__(self, motor):
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
        
        self.motor = motor
        atexit.register(self.motor.all_motors_off) # Should not be needed


        self.timer = self.create_timer(1/REFRESH_RATE, self.controller_update)
        self.latest_rigidbodies_msg = None
        self.latest_markers_msg = None

        # Start Plotting
        # self.get_logger().info("Starting Telematry")
        # plt.ion()
        # self.fig, self.ax = plt.subplots(figsize=(8,6))

        # self.textbox = self.ax.text(
        #     1.05, 0.5,              # x, y position (in axes coordinates)
        #     "",                     # initial text
        #     transform=self.ax.transAxes,  # position relative to axes
        #     fontsize=10,
        #     va='center',
        #     ha='left',
        # )


        
    


    def rigid_bodies_listener_callback(self, msg: RigidBodies):
        self.latest_rigidbodies_msg = msg  # always store the newest message

    def markers_listener_callback(self, msg: Markers):
        self.latest_markers_msg = msg  # always store the newest message
 

        

    def controller_update(self):
        # Get robot pose
        # self.get_logger().info("Updating!")
        robot_body = None
        if self.latest_rigidbodies_msg is not None:
            robot_body = next((rb for rb in self.latest_rigidbodies_msg.rigidbodies if rb.rigid_body_name == '1'), None)
        if robot_body is None:
            self.get_logger().info("Lost Body")
            return
        
        pos = robot_body.pose.position
        ori = robot_body.pose.orientation

        x = pos.x
        y = pos.y

        p = np.array([x,y])


        r_mocap = R.from_quat([ori.x, ori.y, ori.z, ori.w])
        R_correction = R.from_euler('z', -90, degrees=True)
        r_ros = R_correction * r_mocap
        _, _, yaw = r_ros.as_euler('xyz', degrees=False)


        


        # Goal
        x_des, y_des = 0.0, 0.0
        if self.latest_markers_msg is not None:
            goal = next((pt for pt in self.latest_markers_msg.markers if pt.marker_index == GOAL_MARKER_INDEX), None)
            if goal is not None:
                x_des = goal.translation.x
                y_des = goal.translation.y
            else:
                self.get_logger().warn("Lost Goal")
                return
        else:
            self.get_logger().warn("No goal data")
            return
        


        # self.get_logger().info(f"Heading: {str(yaw)}, x: {str(x)}, y: {str(y)}")

        # Compute control signals
        Ux_des = 0
        Uy_des = 0

        dx = x_des - x
        dy = y_des - y

        u = np.array([dx, dy])
        dist_to_goal = np.linalg.norm(u)

        if (dist_to_goal > GOAL_THRESH):
            Ux_des = K_e*(x_des - pos.x)
            Uy_des = K_e*(y_des - pos.y)

        

        theta_des = np.arctan2(Uy_des, Ux_des)
        error_theta = theta_des - yaw
        error_theta_wrapped = np.arctan2(np.sin(error_theta), np.cos(error_theta))

        # self.plot_robot(p, u, yaw, theta_des, error_theta_wrapped)


        S_des = np.sqrt(Ux_des**2 + Uy_des**2)
        S_sat = np.clip(S_des, -S_MAX, S_MAX)

        w_des = 0
        if abs(error_theta_wrapped) > ANGLE_THRESH:
            w_des = K_theta*(error_theta_wrapped) 

        wr_des = (S_sat - L * w_des) / R_wheel
        wl_des = (S_sat + L * w_des) / R_wheel


        maxInput = max(wr_des, wl_des)
        if maxInput > MAX_ACUTUATOR_INPUT:
            #Scale down by same factor
            speed_adjust_factor = MAX_ACUTUATOR_INPUT/maxInput
            wr_des_sat = wr_des * speed_adjust_factor
            wl_des_sat = wl_des * speed_adjust_factor

        else:
            wr_des_sat = wr_des
            wl_des_sat = wl_des


        wr_des_sat_clip = np.clip(wr_des, -MAX_ACUTUATOR_INPUT, MAX_ACUTUATOR_INPUT)
        wl_des_sat_clip = np.clip(wl_des, -MAX_ACUTUATOR_INPUT, MAX_ACUTUATOR_INPUT)

        # Send commands to motors
        self.motor.updateMotorSpeed(wl_des_sat_clip, wr_des_sat_clip)

        # # Log for debugging
        self.get_logger().info(
        #     # f"X={pos.x:.2f}, Y={pos.y:.2f}, Yaw={yaw:.2f} | Goal=({x_des:.2f}, {y_des:.2f}) | S_des={S_sat:.2f}, Theta_des={Theta_des:.2f}, w_des={w_des:.2f} | Cmds L={wl_des:.1f}, R={wr_des:.1f}")
            f"Cmds L={wl_des_sat_clip:.1f}, R={wr_des_sat_clip:.1f}")

        

        

def main(args=None):
    rclpy.init(args=args)
    motor = st.SaberToothMotorDriver(True,True)
    node = ControllerNode(motor)


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
