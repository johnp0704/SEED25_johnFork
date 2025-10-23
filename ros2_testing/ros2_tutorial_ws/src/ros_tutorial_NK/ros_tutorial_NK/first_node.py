#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Basic_Node(Node):
    #Define new class which inherits from RCLPY node
    def __init__(self):
        super().__init__("node_one")
        self.get_logger().info("Starting Node!")
        self.counter = 0
        self.create_timer(1.0, self.timer_callback)
        

    def timer_callback(self):
        self.get_logger().info("Test" + str(self.counter))
        self.counter += 1


 
def main(args = None):
    rclpy.init(args=args)

    testing_node = Basic_Node()

    rclpy.spin(testing_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
    