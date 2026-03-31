import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Pose2D
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
import math

class WaypointGUINode(Node):
    def __init__(self):
        super().__init__('waypoint_gui_node')
        self.get_logger().info("Starting Mission Control GUI with Re-Homing")

        # Publishers
        self.waypoint_pub = self.create_publisher(PoseArray, '/planned_waypoints', 10)
        self.reset_pub = self.create_publisher(Pose2D, '/reset_pose', 10)

        # Subscribers (To show the virtual twin driving live)
        self.create_subscription(Pose2D, '/robot/pose2d', self.pose_callback, 10)

        # State Variables
        self.waypoints = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Room dimensions in meters (7'11" x 11'7")
        self.width_m = 2.413 
        self.height_m = 3.530
        
        # Setup Matplotlib Interactive Window
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 8))
        self.fig.canvas.manager.set_window_title("Mission Control GUI")
        
        # Connect the click event for dropping waypoints
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

        # --- Buttons ---
        # Re-home Button (Bottom Left)
        self.ax_btn_reset = plt.axes([0.15, 0.02, 0.3, 0.05])
        self.btn_reset = Button(self.ax_btn_reset, 'Re-Home Robot', color='lightcoral', hovercolor='salmon')
        self.btn_reset.on_clicked(self.rehome_robot)

        # Send Button (Bottom Right)
        self.ax_btn_send = plt.axes([0.55, 0.02, 0.3, 0.05])
        self.btn_send = Button(self.ax_btn_send, 'Send to Robot', color='lightgreen', hovercolor='palegreen')
        self.btn_send.on_clicked(self.send_waypoints)

        self.draw_map()
        
        # Timer to keep the GUI refreshing at 10Hz to animate the robot
        self.timer = self.create_timer(0.1, self.refresh_gui)

    def pose_callback(self, msg):
        # Continually update the virtual twin's location from the dead-reckoning node
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.theta

    def draw_map(self):
        self.ax.clear()
        
        # Set boundaries (origin 0,0 is the dead center of the room)
        self.ax.set_xlim(-self.width_m / 2, self.width_m / 2)
        self.ax.set_ylim(-self.height_m / 2, self.height_m / 2)
        self.ax.set_title("Click to plot waypoints. (Center is 0,0)")
        self.ax.set_xlabel("Meters")
        self.ax.set_ylabel("Meters")
        self.ax.grid(True, linestyle='--', alpha=0.7)

        # Plot Live Robot Position & Heading
        self.ax.plot(self.robot_x, self.robot_y, 'go', markersize=10, label="Robot Live Pose")
        
        # Draw a little arrow showing which way the robot is facing
        arrow_length = 0.2
        dx = arrow_length * math.cos(self.robot_theta)
        dy = arrow_length * math.sin(self.robot_theta)
        self.ax.arrow(self.robot_x, self.robot_y, dx, dy, head_width=0.05, head_length=0.05, fc='green', ec='green')

        # Plot active Waypoints
        if self.waypoints:
            xs = [self.robot_x] + [w[0] for w in self.waypoints]
            ys = [self.robot_y] + [w[1] for w in self.waypoints]
            self.ax.plot(xs, ys, 'b--o', markersize=6, label="Planned Path")
            
            for i, (x, y) in enumerate(self.waypoints):
                self.ax.text(x + 0.05, y + 0.05, str(i+1), fontsize=10, fontweight='bold', color='blue')

        self.ax.legend(loc="upper right")
        self.fig.canvas.draw()

    def onclick(self, event):
        # Ignore clicks if they fall on our buttons at the bottom
        if event.inaxes != self.ax:
            return
        
        if event.xdata is not None and event.ydata is not None:
            self.waypoints.append((event.xdata, event.ydata))

    def send_waypoints(self, event):
        if not self.waypoints:
            self.get_logger().warn("No waypoints to send!")
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
        self.get_logger().info(f"Deployed {len(self.waypoints)} waypoints!")
        self.waypoints = []

    def rehome_robot(self, event):
        # Blast out a 0,0,0 coordinate to the network
        reset_msg = Pose2D()
        reset_msg.x = 0.0
        reset_msg.y = 0.0
        reset_msg.theta = 0.0
        self.reset_pub.publish(reset_msg)
        
        self.get_logger().info("RE-HOME COMMAND SENT: Dead-reckoning node should reset to origin.")

    def refresh_gui(self):
        # Because we want to watch the robot drive live, we redraw the map on the timer tick
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