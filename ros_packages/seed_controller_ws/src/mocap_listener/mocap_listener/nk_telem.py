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
MAX_ACUTUATOR_INPUT = 20
S_MAX = MAX_ACUTUATOR_INPUT * 0.6
GOAL_THRESH = 0. # m
ANGLE_THRESH = np.deg2rad(10)
REFRESH_RATE = 10 #hz



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
        self.get_logger().info("Starting Telematry")
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8,6))

        self.textbox = self.ax.text(
            1.05, 0.5,              # x, y position (in axes coordinates)
            "",                     # initial text
            transform=self.ax.transAxes,  # position relative to axes
            fontsize=10,
            va='center',
            ha='left',
        )


        
    


    def rigid_bodies_listener_callback(self, msg: RigidBodies):
        self.latest_rigidbodies_msg = msg  # always store the newest message

    def markers_listener_callback(self, msg: Markers):
        self.latest_markers_msg = msg  # always store the newest message

    def plot_robot(self, p, u, yaw, theta_des, angle_error):
        
        for artist in self.ax.lines + self.ax.patches + self.ax.collections:
            artist.remove()
        if self.ax.get_legend():
            self.ax.get_legend().remove()

        goal = p + u


        # Re-plot robot position and heading
        heading_vec = np.array([np.cos(yaw), np.sin(yaw)])
        goal_heading_vec = np.array([np.cos(theta_des), np.sin(theta_des)])

        self.ax.scatter(p[0], p[1], c='r', label='Robot')
        self.ax.scatter(goal[0], goal[1], c='r', label='Goal')
        
        self.ax.arrow(p[0], p[1], heading_vec[0], heading_vec[1], color='b', width=0.02, label = "Heading")
        self.ax.arrow(p[0], p[1], goal_heading_vec[0], goal_heading_vec[1], color='b', width=0.02, label = "Caluclated Goal Heading")
        self.ax.arrow(p[0], p[1], u[0], u[1], color='g', width=0.02, label='Goal vector', length_includes_head=True)

        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.set_aspect('equal', 'box')

        if self.ax.get_legend() is not None:
            print("Clearing Legend!")
            self.ax.get_legend().remove()
        self.ax.legend() 

        self.ax.grid(True)

    

        info = (
            f"X: {p[0]:.2f}\n"
            f"Y: {p[1]:.2f}\n"
            f"Yaw: {np.degrees(yaw):.1f}°\n"
            f"Yaw error: {np.degrees(angle_error):.1f}°\n"
            f"Goal X: {goal[0]:.2f}\n"
            f"Goal Y: {goal[1]:.2f}\n"
            f"Dist: {np.linalg.norm(u):.2f}\n"
            f"Need to turn?: {F>ANGLE_THRESH}"
        )
        self.textbox.set_text(info)

        # Refresh the figure
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        

        

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
            Ux_des = (x_des - pos.x)
            Uy_des = (y_des - pos.y)
            #FIXME add k_e maybe for nice scaling? optional...

        

        theta_des = np.arctan2(Uy_des, Ux_des)
        error_theta = theta_des - yaw
        error_theta_wrapped = np.arctan2(np.sin(error_theta), np.cos(error_theta))

        self.plot_robot(p, u, yaw, theta_des, error_theta_wrapped)

        

        

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
