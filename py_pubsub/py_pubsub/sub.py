import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        # Create the node
        super().__init__("Subscriber_node")
        
        self.subscriber_ = self.create_subscription(String, "TOPIC",self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Subscribing: {msg.data}")


def main(args=None):
    # initialization
    rclpy.init(args=args)

    # Create a subcriber node
    subscriber_node = SubscriberNode()

    # Use the node
    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node
        subscriber_node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()