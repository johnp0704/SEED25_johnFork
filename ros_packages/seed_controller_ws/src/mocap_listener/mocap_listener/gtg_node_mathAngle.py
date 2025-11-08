#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies, Markers
from scipy.spatial.transform import Rotation as R
from mocap_listener import sabertooth as st
import atexit
from mocap_listener import PID as PID
import numpy as np

GOAL_MARKER_INDEX = 5
MAX_ACUTUATOR_INPUT = 25
S_MAX = MAX_ACUTUATOR_INPUT * 0.6
GOAL_THRESH = 0.3

REFRESH_RATE = 10 #hz
R_wheel = .08 #m
L = .178 #m
K_e = 30
K_theta = 50


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
        
    


    def rigid_bodies_listener_callback(self, msg: RigidBodies):
        self.latest_rigidbodies_msg = msg  # always store the newest message

    def markers_listener_callback(self, msg: Markers):
        self.latest_markers_msg = msg  # always store the newest message

        

    def controller_update(self):
    # Get robot pose
        robot_body = None
        if self.latest_rigidbodies_msg is not None:
            robot_body = next((rb for rb in self.latest_rigidbodies_msg.rigidbodies if rb.rigid_body_name == '1'), None)
        if self.latest_markers_msg is None:
            return
        
        if robot_body is None:
            self.get_logger().info("Lost Body")
            return
        
        pos = robot_body.pose.position
        corner_pos = []
        for i in range(5):
            if i != 1:
                corner = next((pt for pt in self.latest_rigidbodies_msg.markers if pt.marker_index == i), None)
                if corner is None:
                    self.get_logger().warn(f"Missing corner marker {i}, dumping!")
                    print(self.latest_markers_msg.markers)
                    return

                x_corner_pos = corner.translation.x
                y_corner_pos = corner.translation.y
                corner_pos.append([x_corner_pos, y_corner_pos])

        x_center = sum([c[0] for c in corner_pos]) / len(corner_pos)
        y_center = sum([c[1] for c in corner_pos]) / len(corner_pos)
        
        x_direction_center = (corner_pos[0][-1] + corner_pos[1][-1]) / 2 #take center of 3rd and 4th croenr points, that the "front" of the robot
        y_direction_center = (corner_pos[0][-2] + corner_pos[1][-2]) / 2

        dx = x_direction_center - x_center
        dy = y_direction_center - y_center
        norm = np.sqrt(dx**2 + dy**2)

        if norm > 1e-8:
            ux = dx / norm
            uy = dy / norm
        else:
            ux, uy = 0.0, 0.0

        unit_vector = (ux, uy)


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

        # Compute control signals
        Ux_des = 0
        Uy_des = 0

        if (np.sqrt((pos.x - x_des)**2 + (pos.y - y_des)**2) > GOAL_THRESH):
            Ux_des = K_e*(x_des - pos.x)
            Uy_des = K_e*(y_des - pos.y)

        S_des = np.sqrt(Ux_des**2 + Uy_des**2)
        S_sat = np.clip(S_des, -S_MAX, S_MAX)

        # Robot heading unit vector
        ux, uy = unit_vector  # from earlier

        # Vector to goal
        gx = x_des - x_center
        gy = y_des - y_center
        g_norm = np.sqrt(gx**2 + gy**2)

        if g_norm > 1e-8:
            gx /= g_norm
            gy /= g_norm
        else:
            gx, gy = 0.0, 0.0

        # Signed angle between vectors
        dot = ux * gx + uy * gy
        cross = ux * gy - uy * gx
        angle = np.arctan2(cross, dot)   # radians, positive = goal to left, negative = goal to right


        w_des = K_theta * angle


        wr_des = (S_sat - L * w_des) / R_wheel
        wl_des = (S_sat + L * w_des) / R_wheel

        # wr_des_sat = np.clip(wr_des, -MAX_ACUTUATOR_INPUT, MAX_ACUTUATOR_INPUT)
        # wl_des_sat = np.clip(wl_des, -MAX_ACUTUATOR_INPUT, MAX_ACUTUATOR_INPUT)

        maxInput = max(wr_des, wl_des)
        if maxInput > MAX_ACUTUATOR_INPUT:
            #Scale down by same factor
            speed_adjust_factor = MAX_ACUTUATOR_INPUT/maxInput
            wr_des_sat = wr_des * speed_adjust_factor
            wl_des_sat = wl_des * speed_adjust_factor


        wr_des_sat_clip = np.clip(wr_des, -MAX_ACUTUATOR_INPUT, MAX_ACUTUATOR_INPUT)
        wl_des_sat_clip = np.clip(wl_des, -MAX_ACUTUATOR_INPUT, MAX_ACUTUATOR_INPUT)

        # Send commands to motors
        self.motor.updateMotorSpeed(wl_des_sat_clip, wr_des_sat_clip)

        # Log for debugging
        self.get_logger().info(
            # f"X={pos.x:.2f}, Y={pos.y:.2f}, Yaw={yaw:.2f} | Goal=({x_des:.2f}, {y_des:.2f}) | S_des={S_sat:.2f}, Theta_des={Theta_des:.2f}, w_des={w_des:.2f} | Cmds L={wl_des:.1f}, R={wr_des:.1f}")
            f"S_des={S_sat:.2f}, Theta_e={angle:.2f}, w_des={w_des:.2f} | Cmds L={wl_des_sat:.1f}, R={wr_des_sat:.1f}")
            # f" | angle_to_goal={angle:.2f}, Theta_goal = {Theta_des:.2f}, Theta_e={error_theta:.2f}, Theta_e_wrap={error_theta_wrapped:.2f}, w_des={w_des:.2f} | Cmds L={wl_des_sat:.1f}, R={wr_des_sat:.1f}")
        

        

def main(args=None):
    rclpy.init(args=args)
    motor = st.SaberToothMotorDriver(True,True)
    node = ControllerNode(motor)

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
