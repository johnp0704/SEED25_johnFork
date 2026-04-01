import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import numpy as np

class VirtualTwinNode(Node):
    def __init__(self):
        super().__init__('virtual_twin_node')
        self.get_logger().info("Starting Virtual Twin Node")

        self.latest_rigidbodies_msg = None
        self.current_goal = None

        # Synchronized with waypoint_manager_node
        self.waypoints = [
            (0.7328, -0.7006),
            (-0.3972, 0.27242),
            (-1.2768, -0.69690),
        ]

        # Subscriptions
        self.create_subscription(RigidBodies, '/rigid_bodies', self.rb_callback, 10)
        self.create_subscription(Point, '/robot/current_target', self.target_callback, 10)

        # Set up interactive plot
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6,6))
        self.create_timer(0.1, self.update_plot)

    def rb_callback(self, msg):
        self.latest_rigidbodies_msg = msg

    def target_callback(self, msg):
        self.current_goal = msg

    def update_plot(self):
        self.ax.clear()
        rb_msg = self.latest_rigidbodies_msg

        # Plot hardcoded waypoints
        xs = [wp[0] for wp in self.waypoints]
        ys = [wp[1] for wp in self.waypoints]
        self.ax.scatter(xs, ys, marker='x', color='gray', label='Waypoints')
        
        for i, (x, y) in enumerate(self.waypoints):
            self.ax.text(x + 0.05, y + 0.05, f"{i}", fontsize=9)

        # Plot robot using Rigid Body Position and Orientation
        if rb_msg:
            # Fixed Rigid Body Name to '2'
            robot_body = next((rb for rb in rb_msg.rigidbodies if rb.rigid_body_name == '2'), None)
            
            if robot_body:
                pos = robot_body.pose.position
                ori = robot_body.pose.orientation

                r_mocap = R.from_quat([ori.x, ori.y, ori.z, ori.w])
                R_correction = R.from_euler('z', -90, degrees=True)
                r_ros = R_correction * r_mocap
                _, _, yaw = r_ros.as_euler('xyz', degrees=False)

                # Plot Robot Center
                self.ax.scatter(pos.x, pos.y, marker='o', s=100, color='green', label='Robot Center')

                # Draw Heading arrow
                arrow_length = 0.3
                dx = arrow_length * np.cos(yaw)
                dy = arrow_length * np.sin(yaw)
                self.ax.arrow(pos.x, pos.y, dx, dy, head_width=0.05, fc='blue', ec='blue', label='Heading')

                # Dynamic Line to active goal
                if self.current_goal:
                    self.ax.plot([pos.x, self.current_goal.x],
                                 [pos.y, self.current_goal.y],
                                 'r--', label='Active Path')

        self.ax.set_xlabel('x (m)')
        self.ax.set_ylabel('y (m)')
        self.ax.set_title('Virtual Twin & Active Path')
        self.ax.axis('equal')
        
        # Prevent duplicate labels in legend
        handles, labels = self.ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        self.ax.legend(by_label.values(), by_label.keys(), loc='upper left')
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = VirtualTwinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()