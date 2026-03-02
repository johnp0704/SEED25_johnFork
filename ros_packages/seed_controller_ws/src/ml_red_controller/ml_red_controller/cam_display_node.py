import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/annotated_image',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message back to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        cv2.imshow("YOLO Live Feed", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()