import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__("Publisher_Node")
        self.publisher_ = self.create_publisher(String, "TOPIC", 10)
        timer_period = 0.5 # second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.i = 0

    def timer_callback(self):
        # Initialize the publishing message
        message = String()
        message.data = f"Hello World: {self.i}"

        # Publishing the message
        self.publisher_.publish(message)

        # Log print the message
        self.get_logger().info(f"Publishing: {message.data}")

        self.i += 1


# Create the main function
def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    # Create a Node
    publisher_node = PublisherNode()

    try:
        # Use the node
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
          pass # Do nothing, just let it move on to the destroy steps
    
    finally:
          # Destroy the Node
          publisher_node.destroy_node()
    
    # Shutdown the system
    if rclpy.ok():
          rclpy.shutdown()


if __name__ == '__main__':
    main()