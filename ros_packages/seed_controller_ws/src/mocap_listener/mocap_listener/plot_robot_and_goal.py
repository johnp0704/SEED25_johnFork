#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import tf.transformations as tft

robot_pos = np.array([0, 0])
robot_yaw = 0.0
goal = None

def odom_callback(msg):
    global robot_pos, robot_yaw
    robot_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    q = msg.pose.pose.orientation
    robot_yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

def goal_callback(msg):
    global goal
    goal = np.array([msg.point.x, msg.point.y])

if __name__ == "__main__":
    rospy.init_node("live_robot_plotter")
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/goal", PointStamped, goal_callback)

    plt.ion()
    fig, ax = plt.subplots()

    while not rospy.is_shutdown():
        ax.clear()
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(robot_pos[0] - 2, robot_pos[0] + 2)
        ax.set_ylim(robot_pos[1] - 2, robot_pos[1] + 2)

        # Draw robot
        ax.plot(robot_pos[0], robot_pos[1], 'bo', label="Robot")
        ax.arrow(robot_pos[0], robot_pos[1], 0.3 * np.cos(robot_yaw), 0.3 * np.sin(robot_yaw),
                 head_width=0.1, color='blue')

        # Draw goal
        if goal is not None:
            ax.plot(goal[0], goal[1], 'rx', markersize=12, label="Goal")

        ax.legend()
        plt.pause(0.01)
