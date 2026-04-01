import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Pose2D
from std_msgs.msg import Empty
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
import math

class WaypointGUINode(Node):
    def __init__(self):
        super().__init__('waypoint_gui_node')
        self.get_logger().info("Starting Mission Control GUI")

        self.waypoint_pub = self.create_publisher(PoseArray, '/planned_waypoints', 10)
        self.reset_pub = self.create_publisher(Pose2D, '/reset_pose', 10)
        self.calib_pub = self.create_publisher(Empty, '/start_calibration', 10)

        self.create_subscription(Pose2D, '/robot/pose2d', self.pose_callback, 10)

        self.waypoints = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.width_m = 2.413 
        self.height_m = 3.530
        
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 8))
        self.fig.canvas.manager.set_window_title("Mission Control GUI")
        
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

        # --- Re-Home Button (Left) ---
        self.ax_btn_reset = plt.axes([0.05, 0.02, 0.25, 0.05])
        self.btn_reset = Button(self.ax_btn_reset, 'Re-Home', color='lightcoral', hovercolor='salmon')
        self.btn_reset.on_clicked(self.rehome_robot)

        # --- Calibrate Button (Center) ---
        self.ax_btn_calib = plt.axes([0.35, 0.02, 0.25, 0.05])
        self.btn_calib = Button(self.ax_btn_calib, 'Calibrate', color='khaki', hovercolor='gold')
        self.btn_calib.on_clicked(self.trigger_calibration)

        # --- Send Button (Right) ---
        self.ax_btn_send = plt.axes([0.65, 0.02, 0.25, 0.05])
        self.btn_send = Button(self.ax_btn_send, 'Send Path', color='lightgreen', hovercolor='palegreen')
        self.btn_send.on_clicked(self.send_waypoints)

        self.draw_map()
        self.timer = self.create_timer(0.1, self.refresh_gui)

    def pose_callback(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.theta

    def draw_map(self):
        self.ax.clear()
        self.ax.set_xlim(-self.width_m / 2, self.width_m / 2)
        self.ax.set_ylim(-self.height_m / 2, self.height_m / 2)
        self.ax.set_title("Click to plot waypoints. (Center is 0,0)")
        self.ax.set_xlabel("Meters")
        self.ax.set_ylabel("Meters")
        self.ax.grid(True, linestyle='--', alpha=0.7)

        self.ax.plot(self.robot_x, self.robot_y, 'go', markersize=10, label="Robot Live Pose")
        
        arrow_length = 0.2
        dx = arrow_length * math.cos(self.robot_theta)
        dy = arrow_length * math.sin(self.robot_theta)
        self.ax.arrow(self.robot_x, self.robot_y, dx, dy, head_width=0.05, head_length=0.05, fc='green', ec='green')

        if self.waypoints:
            xs = [self.robot_x] + [w[0] for w in self.waypoints]
            ys = [self.robot_y] + [w[1] for w in self.waypoints]
            self.ax.plot(xs, ys, 'b--o', markersize=6, label="Planned Path")
            
            for i, (x, y) in enumerate(self.waypoints):
                self.ax.text(x + 0.05, y + 0.05, str(i+1), fontsize=10, fontweight='bold', color='blue')

        self.ax.legend(loc="upper right")
        self.fig.canvas.draw()

    def onclick(self, event):
        if event.inaxes != self.ax:
            return
        if event.xdata is not None and event.ydata is not None:
            self.waypoints.append((event.xdata, event.ydata))

    def send_waypoints(self, event):
        if not self.waypoints:
            return

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        for wp in self.waypoints:
            p = Pose()
            p.position.x = wp[0]
            p.position.y = wp[1]
            p.position.z = 0.0
            msg.poses.append(p)

        self.waypoint_pub.publish(msg)
        self.waypoints = []

    def rehome_robot(self, event):
        reset_msg = Pose2D()
        reset_msg.x = 0.0
        reset_msg.y = 0.0
        reset_msg.theta = 0.0
        self.reset_pub.publish(reset_msg)

    def trigger_calibration(self, event):
        self.calib_pub.publish(Empty())

    def refresh_gui(self):
        self.draw_map()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointGUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        plt.close('all')
        rclpy.shutdown()

if __name__ == '__main__':
    main()