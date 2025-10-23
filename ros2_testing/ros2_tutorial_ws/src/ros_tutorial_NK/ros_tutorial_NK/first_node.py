#!/user/bin/env python3
import rclpy
from rclpy.node import Node

class Basic_Node(Node):
    #Define new class which inherits from RCLPY node
    def __init__(self):
        self.__init__("node_one")
        self.get_logger().info("test!")



def main(args = None):
    rclpy.init(args=args)

    testing_node = Basic_Node()







    rclpy.shutdown()


if __name__ == "__main__":
    main()
    