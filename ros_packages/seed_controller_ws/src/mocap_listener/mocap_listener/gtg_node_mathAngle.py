#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from mocap4r2_msgs.msg import RigidBodies, Markers
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from mocap_listener import sabertooth as st
import atexit
import numpy as np
import time
import threading
import matplotlib.pyplot as plt

# Constants
GOAL_MARKER_INDEX = 5
MAX_ACTUATOR_INPUT = 25.0
S_MAX = MAX_ACTUATOR_INPUT * 0.6
GOAL_THRESH = 0.3
REFRESH_RATE = 10.0  # Hz
R_wheel = 0.08  # m
L = 0.178  # m

def angle_normalize(a):
    return np.arctan2(np.sin(a), np.cos(a))

class ControllerNode(Node):
    def __init__(self, motor):
        super().__init__('controller_node')
        self.get_logger().info("Starting GTG Controller node (heading 0-3 normal, goal alignment first)")

        cb_group = ReentrantCallbackGroup()
        self.subscription_rb = self.create_subscription(
            RigidBodies, '/rigid_bodies', self.rigid_bodies_listener_callback,
            int(REFRESH_RATE), callback_group=cb_group
        )
        self.subscription_markers = self.create_subscription(
            Markers, '/markers', self.markers_listener_callback,
            int(REFRESH_RATE), callback_group=cb_group
        )
        self.pose_pub = self.create_publisher(Pose2D, '/controller/pose2d', 10)
        self.markers_pub = self.create_publisher(Float32MultiArray, '/controller/marker_array', 10)
        self.create_service(Trigger, 'dump_marker_plot', self.handle_dump_marker_plot, callback_group=cb_group)

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
                ('speed_ramp_rate', float(50.0)),
                ('forward_angle_slowing', float(0.7)),
                ('min_forward_scale', float(0.05)),
            ]
        )
        self._refresh_local_params()
        self.add_on_set_parameters_callback(self._param_change_callback)

        self.motor = motor
        atexit.register(self.motor.all_motors_off)

        self.prev_wl = 0.0
        self.prev_wr = 0.0
        self.prev_angle = 0.0
        self.prev_time = time.time()

        self.latest_rigidbodies_msg = None
        self.latest_markers_msg = None
        self._msg_lock = threading.Lock()

        period = 1.0 / self.refresh_rate
        self.timer = self.create_timer(period, self.controller_update, callback_group=cb_group)

    def _refresh_local_params(self):
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

        self.MAX_ACUTUATOR_INPUT = self.max_actuator_input
        self.S_MAX = self.s_max

    def _param_change_callback(self, params):
        for param in params:
            if param.name not in ['goal_marker_index','max_actuator_input','s_max_scale','goal_thresh','refresh_rate',
                                  'r_wheel','L','K_e','K_theta_p','K_theta_d','angle_sat','angle_deadzone',
                                  'speed_ramp_rate','forward_angle_slowing','min_forward_scale']:
                return rclpy.parameter.SetParametersResult(successful=False, reason=f"Unknown param {param.name}")
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
        with self._msg_lock:
            markers_msg = self.latest_markers_msg
            rb_msg = self.latest_rigidbodies_msg
        if markers_msg is None and rb_msg is None:
            response.success = False
            response.message = "No marker/rigidbody data received yet."
            return response

        fig, ax = plt.subplots(figsize=(6,6))
        if markers_msg:
            xs = [m.translation.x for m in markers_msg.markers]
            ys = [m.translation.y for m in markers_msg.markers]
            ax.scatter(xs, ys, marker='x', label='all markers')
            for m in markers_msg.markers:
                ax.text(m.translation.x, m.translation.y, f"{m.marker_index}", fontsize=8)
        if rb_msg:
            robot_body = next((rb for rb in rb_msg.rigidbodies if rb.rigid_body_name=='1'), None)
            if robot_body:
                corner_pos=[]
                for i in [0,2,3,4]:
                    corner = next((pt for pt in robot_body.markers if pt.marker_index==i), None)
                    if corner:
                        corner_pos.append((corner.translation.x, corner.translation.y))
                if corner_pos:
                    cp = np.array(corner_pos)
                    ax.scatter(cp[:,0], cp[:,1], marker='o', label='robot corners')
                    for idx,(xx,yy) in enumerate(cp):
                        ax.text(xx,yy,f"corner_{idx}",fontsize=8,color='green')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title('Latest Markers and Robot Corners')
        ax.legend()
        ax.axis('equal')
        outpath = '/tmp/controller_marker_dump.png'
        try:
            fig.savefig(outpath)
            plt.close(fig)
            response.success=True
            response.message=f"Saved marker plot to {outpath}"
        except Exception as e:
            response.success=False
            response.message=f"Failed to save plot: {e}"
        return response

    def controller_update(self):
        with self._msg_lock:
            rb_msg = self.latest_rigidbodies_msg
            markers_msg = self.latest_markers_msg
        if rb_msg is None:
            return
        robot_body = next((rb for rb in rb_msg.rigidbodies if rb.rigid_body_name=='1'), None)
        if robot_body is None:
            self.get_logger().info("Lost Body")
            return

        corner_pos = []
        for i in [0,2,3,4]:
            corner = next((pt for pt in robot_body.markers if pt.marker_index==i), None)
            if corner is None:
                self.get_logger().warn(f"Missing corner {i}")
                return
            corner_pos.append([corner.translation.x, corner.translation.y])

        x_center = sum(c[0] for c in corner_pos)/len(corner_pos)
        y_center = sum(c[1] for c in corner_pos)/len(corner_pos)

        # Heading: midpoint between 0 and 3, normal to line, **reversed**
        x0,y0 = corner_pos[0]
        x3,y3 = corner_pos[2]
        dx = x3 - x0
        dy = y3 - y0
        ux = dy   # reversed
        uy = -dx  # reversed
        norm = np.sqrt(ux**2 + uy**2)
        if norm>1e-8:
            ux /= norm
            uy /= norm
        else:
            ux=uy=0.0

        # goal
        x_des = y_des = 0.0
        if markers_msg:
            goal = next((pt for pt in markers_msg.markers if pt.marker_index==int(self.goal_marker_index)), None)
            if goal:
                x_des = goal.translation.x
                y_des = goal.translation.y
            else:
                self.get_logger().warn("Lost Goal marker")
                return
        else:
            self.get_logger().warn("No goal data")
            return

        # vector to goal
        gx = x_des - x_center
        gy = y_des - y_center
        g_norm = np.sqrt(gx**2 + gy**2)
        if g_norm>1e-8:
            gx /= g_norm
            gy /= g_norm
        else:
            gx=gy=0.0

        # Angle difference
        dot = ux*gx + uy*gy
        cross = ux*gy - uy*gx
        angle = angle_normalize(np.arctan2(cross, dot))
        if abs(abs(angle)-np.pi)<0.02:
            angle = np.sign(angle)*(np.pi-0.02)

        now = time.time()
        dt = max(1e-6, now-self.prev_time)
        d_angle = (angle - self.prev_angle)/dt if dt>0 else 0.0

        # Phase 1: rotate to align with goal
        if abs(angle) > self.angle_deadzone:
            S_sat = 0.0  # don't move forward yet
            w_des = (self.K_theta_p*angle + self.K_theta_d*d_angle)
        else:
            # Phase 2: move straight toward goal
            dist_to_goal = np.sqrt((x_center - x_des)**2 + (y_center - y_des)**2)
            if dist_to_goal > self.goal_thresh:
                S_des = self.K_e*dist_to_goal
                forward_scale = max(self.min_forward_scale, np.cos(angle)*self.forward_angle_slowing)
                S_sat = np.clip(S_des, -self.S_MAX, self.S_MAX) * forward_scale
            else:
                S_sat = 0.0
            w_des = 0.0

        wr_des = (S_sat - self.L*w_des)/self.r_wheel
        wl_des = (S_sat + self.L*w_des)/self.r_wheel

        wr_des_sat = np.clip(wr_des, -self.MAX_ACUTUATOR_INPUT, self.MAX_ACUTUATOR_INPUT)
        wl_des_sat = np.clip(wl_des, -self.MAX_ACUTUATOR_INPUT, self.MAX_ACUTUATOR_INPUT)

        max_delta = self.speed_ramp_rate*dt
        delta_l = wl_des_sat - self.prev_wl
        delta_r = wr_des_sat - self.prev_wr
        if abs(delta_l)>max_delta:
            wl_des_sat = self.prev_wl + np.sign(delta_l)*max_delta
        if abs(delta_r)>max_delta:
            wr_des_sat = self.prev_wr + np.sign(delta_r)*max_delta

        try:
            self.motor.updateMotorSpeed(float(wl_des_sat), float(wr_des_sat))
        except Exception as e:
            self.get_logger().error(f"Motor update error: {e}")

        self.prev_wl = wl_des_sat
        self.prev_wr = wr_des_sat
        self.prev_angle = angle
        self.prev_time = now

        pose = Pose2D()
        pose.x = float(x_center)
        pose.y = float(y_center)
        pose.theta = float(np.arctan2(uy, ux))
        self.pose_pub.publish(pose)

        arr = Float32MultiArray()
        if markers_msg:
            data=[]
            for m in markers_msg.markers:
                data.extend([float(m.marker_index), float(m.translation.x), float(m.translation.y)])
            arr.data=data
            self.markers_pub.publish(arr)

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

if __name__=='__main__':
    main()
