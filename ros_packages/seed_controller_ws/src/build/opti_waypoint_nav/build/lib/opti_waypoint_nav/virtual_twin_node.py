import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies, Markers
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import numpy as np

class VirtualTwinNode(Node):
    def __init__(self):
        super().__init__('virtual_twin_node')
        self.get_logger().info("Starting Virtual Twin Node")

        self.latest_rigidbodies_msg = None
        self.latest_markers_msg = None
        self.current_goal = None

        # Subscriptions
        self.create_subscription(RigidBodies, '/rigid_bodies', self.rb_callback, 10)
        self.create_subscription(Markers, '/markers', self.marker_callback, 10)
        self.create_subscription(Point, '/robot/current_target', self.target_callback, 10)

        # Set up interactive plot
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6,6))
        self.create_timer(0.1, self.update_plot)

    def rb_callback(self, msg):
        self.latest_rigidbodies_msg = msg

    def marker_callback(self, msg):
        self.latest_markers_msg = msg

    def target_callback(self, msg):
        self.current_goal = msg

    def update_plot(self):
        self.ax.clear()
        rb_msg = self.latest_rigidbodies_msg
        markers_msg = self.latest_markers_msg

        # Plot all markers
        if markers_msg:
            xs = [m.translation.x for m in markers_msg.markers]
            ys = [m.translation.y for m in markers_msg.markers]
            self.ax.scatter(xs, ys, marker='x', color='gray', label='Waypoints')
            for m in markers_msg.markers:
                self.ax.text(m.translation.x, m.translation.y, f"{m.marker_index}", fontsize=8)

        # Plot robot
        if rb_msg:
            robot_body = next((rb for rb in rb_msg.rigidbodies if rb.rigid_body_name == '1'), None)
            if robot_body:
                corner_pos = []
                for i in [0,2,3,4]:
                    corner = next((pt for pt in robot_body.markers if pt.marker_index == i), None)
                    if corner:
                        corner_pos.append((corner.translation.x, corner.translation.y))
                if corner_pos:
                    cp = np.array(corner_pos)
                    self.ax.scatter(cp[:,0], cp[:,1], marker='o', color='green', label='Robot Corners')
                    
                    x_center = np.mean(cp[:,0])
                    y_center = np.mean(cp[:,1])
                    x_front = (cp[1,0] + cp[2,0])/2.0
                    y_front = (cp[1,1] + cp[2,1])/2.0

                    # Heading arrow
                    dx = x_center - x_front
                    dy = y_center - y_front
                    self.ax.arrow(x_center, y_center, dx, dy, head_width=0.02, fc='blue', ec='blue', label='Heading')

                    # Dynamic Line to active goal
                    if self.current_goal:
                        self.ax.plot([x_front, self.current_goal.x],
                                     [y_front, self.current_goal.y],
                                     'r--', label='Active Path')

        self.ax.set_xlabel('x (m)')
        self.ax.set_ylabel('y (m)')
        self.ax.set_title('Virtual Twin & Active Path')
        self.ax.axis('equal')
        self.ax.legend()
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