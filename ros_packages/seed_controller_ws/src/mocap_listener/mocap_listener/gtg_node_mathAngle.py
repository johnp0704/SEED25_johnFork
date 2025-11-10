#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from mocap4r2_msgs.msg import RigidBodies, Markers
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation as R
from mocap_listener import sabertooth as st
import atexit
import numpy as np
import time
import threading
import matplotlib.pyplot as plt
import os

# Constants (default values are also set as ROS parameters below)
GOAL_MARKER_INDEX = 5
MAX_ACTUATOR_INPUT = 25.0
S_MAX = MAX_ACTUATOR_INPUT * 0.6
GOAL_THRESH = 0.3

REFRESH_RATE = 10.0  # Hz
R_wheel = 0.08  # m
L = 0.178  # m

# Utility
def angle_normalize(a):
    """Normalize angle to [-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


class ControllerNode(Node):
    def __init__(self, motor):
        super().__init__('controller_node')

        self.get_logger().info("Starting GTG Controller node (improved)")

        # Callback group so parameter updates and callbacks can interleave safely
        cb_group = ReentrantCallbackGroup()

        # Subscriptions to mocap topics
        self.subscription_rb = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.rigid_bodies_listener_callback,
            int(REFRESH_RATE),
            callback_group=cb_group
        )
        self.subscription_markers = self.create_subscription(
            Markers,
            '/markers',
            self.markers_listener_callback,
            int(REFRESH_RATE),
            callback_group=cb_group
        )

        # Publishers: pose and raw marker positions for plotting/debug
        self.pose_pub = self.create_publisher(Pose2D, '/controller/pose2d', 10)
        self.markers_pub = self.create_publisher(Float32MultiArray, '/controller/marker_array', 10)

        # Service to dump a plot of the latest markers to a PNG for indexing check
        self.create_service(Trigger, 'dump_marker_plot', self.handle_dump_marker_plot, callback_group=cb_group)

        # Parameters (tunable live)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('goal_marker_index', int(GOAL_MARKER_INDEX)),
                ('max_actuator_input', float(MAX_ACTUATOR_INPUT)),
                ('s_max_scale', float(0.6)),
                ('goal_thresh', float(GOAL_THRESH)),
                ('refresh_rate', float(REFRESH_RATE)),
                ('r_wheel', float(R_wheel)),
                ('L', float(L)),
                ('K_e', float(30.0)),
                ('K_theta_p', float(50.0)),
                ('K_theta_d', float(0.0)),
                ('angle_sat', float(0.5)),
                ('angle_deadzone', float(0.05)),
                ('speed_ramp_rate', float(50.0)),  # max actuator units per second change
                ('forward_angle_slowing', float(0.7)),  # how much to reduce forward speed at large angle (0..1)
                ('min_forward_scale', float(0.05)),  # minimum forward speed scale when turning
            ]
        )

        # storage for param values (and convenience variables)
        self._refresh_local_params()

        # watch for parameter changes (live tuning)
        self.add_on_set_parameters_callback(self._param_change_callback)

        # motor driver
        self.motor = motor
        atexit.register(self.motor.all_motors_off)

        # internal state for ramping & angle derivative
        self.prev_wl = 0.0
        self.prev_wr = 0.0
        self.prev_angle = 0.0
        self.prev_time = time.time()

        # latest messages and thread lock
        self.latest_rigidbodies_msg = None
        self.latest_markers_msg = None
        self._msg_lock = threading.Lock()

        # timer for control loop
        period = 1.0 / self.refresh_rate
        self.timer = self.create_timer(period, self.controller_update, callback_group=cb_group)

    def _refresh_local_params(self):
        # Read parameters into local variables
        p = self.get_parameters
        self.goal_marker_index = self.get_parameter('goal_marker_index').get_parameter_value().integer_value
        self.max_actuator_input = self.get_parameter('max_actuator_input').get_parameter_value().double_value
        self.s_max_scale = self.get_parameter('s_max_scale').get_parameter_value().double_value
        self.s_max = self.max_actuator_input * self.s_max_scale
        self.goal_thresh = self.get_parameter('goal_thresh').get_parameter_value().double_value
        self.refresh_rate = self.get_parameter('refresh_rate').get_parameter_value().double_value
        self.r_wheel = self.get_parameter('r_wheel').get_parameter_value().double_value
        self.L = self.get_parameter('L').get_parameter_value().double_value
        self.K_e = self.get_parameter('K_e').get_parameter_value().double_value
        self.K_theta_p = self.get_parameter('K_theta_p').get_parameter_value().double_value
        self.K_theta_d = self.get_parameter('K_theta_d').get_parameter_value().double_value
        self.angle_sat = self.get_parameter('angle_sat').get_parameter_value().double_value
        self.angle_deadzone = self.get_parameter('angle_deadzone').get_parameter_value().double_value
        self.speed_ramp_rate = self.get_parameter('speed_ramp_rate').get_parameter_value().double_value
        self.forward_angle_slowing = self.get_parameter('forward_angle_slowing').get_parameter_value().double_value
        self.min_forward_scale = self.get_parameter('min_forward_scale').get_parameter_value().double_value

        # recompute derived limits
        self.MAX_ACUTUATOR_INPUT = self.max_actuator_input
        self.S_MAX = self.s_max

    def _param_change_callback(self, params):
        # When parameters are changed at runtime via `ros2 param set`, refresh local values.
        for param in params:
            name = param.name
            # accept all declared params
            if name in ['goal_marker_index', 'max_actuator_input', 's_max_scale', 'goal_thresh', 'refresh_rate',
                        'r_wheel', 'L', 'K_e', 'K_theta_p', 'K_theta_d', 'angle_sat', 'angle_deadzone',
                        'speed_ramp_rate', 'forward_angle_slowing', 'min_forward_scale']:
                continue
            # reject unknown param
            return rclpy.parameter.SetParametersResult(successful=False, reason=f"Unknown param {name}")
        # If we reach here, accept and refresh
        self._refresh_local_params()
        self.get_logger().info("Parameters updated live")
        return rclpy.parameter.SetParametersResult(successful=True)

    def rigid_bodies_listener_callback(self, msg: RigidBodies):
        with self._msg_lock:
            self.latest_rigidbodies_msg = msg

    def markers_listener_callback(self, msg: Markers):
        with self._msg_lock:
            self.latest_markers_msg = msg

    def handle_dump_marker_plot(self, request, response):
        """Service handler that writes a PNG of the most recent marker geometry for offline inspection."""
        with self._msg_lock:
            markers_msg = self.latest_markers_msg
            rb_msg = self.latest_rigidbodies_msg

        if markers_msg is None and rb_msg is None:
            response.success = False
            response.message = "No marker/rigidbody data received yet."
            return response

        fig, ax = plt.subplots(figsize=(6, 6))
        if markers_msg:
            xs = [m.translation.x for m in markers_msg.markers]
            ys = [m.translation.y for m in markers_msg.markers]
            ax.scatter(xs, ys, marker='x', label='all markers')
            for m in markers_msg.markers:
                ax.text(m.translation.x, m.translation.y, f"{m.marker_index}", fontsize=8)
        if rb_msg:
            # try to plot the robot corners of body '1'
            robot_body = next((rb for rb in rb_msg.rigidbodies if rb.rigid_body_name == '1'), None)
            if robot_body:
                corner_pos = []
                for i in [0, 2, 3, 4]:
                    corner = next((pt for pt in robot_body.markers if pt.marker_index == i), None)
                    if corner:
                        corner_pos.append((corner.translation.x, corner.translation.y))
                if corner_pos:
                    cp = np.array(corner_pos)
                    ax.scatter(cp[:, 0], cp[:, 1], marker='o', label='robot corners')
                    for idx, (xx, yy) in enumerate(cp):
                        ax.text(xx, yy, f"corner_{idx}", fontsize=8, color='green')

        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title('Latest Markers and Robot Corners (index labels)')
        ax.legend()
        ax.axis('equal')
        # Save file
        outpath = '/tmp/controller_marker_dump.png'
        try:
            fig.savefig(outpath)
            plt.close(fig)
            response.success = True
            response.message = f"Saved marker plot to {outpath}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to save plot: {e}"
        return response

    def controller_update(self):
        # lock and copy messages
        with self._msg_lock:
            rb_msg = self.latest_rigidbodies_msg
            markers_msg = self.latest_markers_msg

        if rb_msg is None:
            # nothing to do
            return

        # find the robot rigid body (name '1' as original code)
        robot_body = next((rb for rb in rb_msg.rigidbodies if rb.rigid_body_name == '1'), None)
        if robot_body is None:
            self.get_logger().info("Lost Body")
            return

        # Get corner positions (original indices: 0,2,3,4)
        corner_pos = []
        for i in [0, 2, 3, 4]:
            corner = next((pt for pt in robot_body.markers if pt.marker_index == i), None)
            if corner is None:
                self.get_logger().warn(f"Missing corner marker {i}, dumping!")
                # publish markers for debugging
                if markers_msg:
                    arr = Float32MultiArray()
                    arr.data = [m.marker_index for m in markers_msg.markers]
                    self.markers_pub.publish(arr)
                return
            corner_pos.append([corner.translation.x, corner.translation.y])

        # Compute robot center
        x_center = sum(c[0] for c in corner_pos) / len(corner_pos)
        y_center = sum(c[1] for c in corner_pos) / len(corner_pos)

        # Front of robot using markers 2 and 3 (correct indices)
        x_front = (corner_pos[1][0] + corner_pos[2][0]) / 2.0
        y_front = (corner_pos[1][1] + corner_pos[2][1]) / 2.0

        # Heading vector (unit)
        dx = x_front - x_center
        dy = y_front - y_center
        norm = np.sqrt(dx ** 2 + dy ** 2)
        if norm > 1e-8:
            ux = dx / norm
            uy = dy / norm
        else:
            ux, uy = 0.0, 0.0

        # Get goal position from markers topic
        x_des, y_des = 0.0, 0.0
        if markers_msg:
            goal = next((pt for pt in markers_msg.markers if pt.marker_index == int(self.goal_marker_index)), None)
            if goal is not None:
                x_des = goal.translation.x
                y_des = goal.translation.y
            else:
                self.get_logger().warn("Lost Goal marker")
                return
        else:
            self.get_logger().warn("No goal data")
            return

        # Compute linear control signals from robot center (P-control)
        dist_to_goal = np.sqrt((x_center - x_des) ** 2 + (y_center - y_des) ** 2)
        if dist_to_goal > self.goal_thresh:
            Ux_des = self.K_e * (x_des - x_center)
            Uy_des = self.K_e * (y_des - y_center)
        else:
            Ux_des = 0.0
            Uy_des = 0.0

        S_des = np.sqrt(Ux_des ** 2 + Uy_des ** 2)
        # clip magnitude
        S_sat = np.clip(S_des, -self.S_MAX, self.S_MAX)

        # Vector to goal (unit)
        gx = x_des - x_center
        gy = y_des - y_center
        g_norm = np.sqrt(gx ** 2 + gy ** 2)
        if g_norm > 1e-8:
            gx /= g_norm
            gy /= g_norm
        else:
            gx, gy = 0.0, 0.0

        # Angle between heading and goal (use normalized vector math)
        dot = ux * gx + uy * gy
        cross = ux * gy - uy * gx
        raw_angle = np.arctan2(cross, dot)
        angle = angle_normalize(raw_angle)

        # Avoid the unstable behavior at ±pi by nudging angle away from exact pi
        if abs(abs(angle) - np.pi) < 0.02:
            # nudge a little toward zero keeping sign
            angle = np.sign(angle) * (np.pi - 0.02)

        # Angular PD controller with deadzone + soft-scaling near zero to reduce stutter
        now = time.time()
        dt = max(1e-6, now - self.prev_time)
        angle_error = angle
        # derivative
        d_angle = (angle - self.prev_angle) / dt if dt > 0 else 0.0

        # deadzone around angle = 0
        if abs(angle_error) < self.angle_deadzone:
            w_des = 0.0
        else:
            # soft-scaling near zero to reduce stutter: scale factor between 0 and 1
            angle_mag = abs(angle_error)
            if angle_mag < self.angle_sat:
                scale = (angle_mag / self.angle_sat)
            else:
                scale = 1.0
            # PD control
            w_des = (self.K_theta_p * angle_error + self.K_theta_d * d_angle) * scale

        # reduce forward speed when large turning necessary
        # use cosine-based scaling with floor
        forward_scale = max(self.min_forward_scale, np.cos(angle) * self.forward_angle_slowing)
        S_sat *= forward_scale

        # Compute wheel commands (convert linear/angular to wheel speeds)
        wr_des = (S_sat - self.L * w_des) / self.r_wheel
        wl_des = (S_sat + self.L * w_des) / self.r_wheel

        # Clip wheel speeds to actuator range
        wr_des_sat = np.clip(wr_des, -self.MAX_ACUTUATOR_INPUT, self.MAX_ACUTUATOR_INPUT)
        wl_des_sat = np.clip(wl_des, -self.MAX_ACUTUATOR_INPUT, self.MAX_ACUTUATOR_INPUT)

        # Ramping / slew-rate limiter: limit change per second
        max_delta_per_sec = float(self.speed_ramp_rate)
        max_delta = max_delta_per_sec * dt
        # apply to left
        delta_l = wl_des_sat - self.prev_wl
        if abs(delta_l) > max_delta:
            wl_des_sat = self.prev_wl + np.sign(delta_l) * max_delta
        # apply to right
        delta_r = wr_des_sat - self.prev_wr
        if abs(delta_r) > max_delta:
            wr_des_sat = self.prev_wr + np.sign(delta_r) * max_delta

        # Send motor commands
        try:
            self.motor.updateMotorSpeed(float(wl_des_sat), float(wr_des_sat))
        except Exception as e:
            self.get_logger().error(f"Motor update error: {e}")

        # Save states
        self.prev_wl = wl_des_sat
        self.prev_wr = wr_des_sat
        self.prev_angle = angle
        self.prev_time = now

        # Publish pose and marker array for visualization/debugging
        pose = Pose2D()
        pose.x = float(x_center)
        pose.y = float(y_center)
        # heading as yaw from heading vector (ux,uy)
        pose.theta = float(np.arctan2(uy, ux))
        self.pose_pub.publish(pose)

        # publish markers as flattened array: [index, x, y, index, x, y, ...]
        arr = Float32MultiArray()
        if markers_msg:
            data = []
            for m in markers_msg.markers:
                data.extend([float(m.marker_index), float(m.translation.x), float(m.translation.y)])
            arr.data = data
            self.markers_pub.publish(arr)

        # Logging — keep concise, but useful for debugging
        self.get_logger().info(
            f"S_des={S_sat:.2f}, Theta_e={angle:.3f}, w_des={w_des:.2f} | Cmds L={wl_des_sat:.1f}, R={wr_des_sat:.1f}"
        )


def main(args=None):
    rclpy.init(args=args)
    motor = st.SaberToothMotorDriver(True, True)
    node = ControllerNode(motor)

    atexit.register(motor.all_motors_off)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exiting")
        motor.all_motors_off()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
