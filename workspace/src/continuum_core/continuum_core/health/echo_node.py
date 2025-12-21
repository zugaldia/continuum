import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class EchoNode(Node):
    def __init__(self):
        super().__init__("echo_node")
        self.create_subscription(String, "topic", self.listener_callback, 10)
        self.get_logger().info("Echo node initialized.")

    def listener_callback(self, msg):
        self.get_logger().info(f"Echo: {msg.data}")


def main(args=None):
    try:
        with rclpy.init(args=args):
            echo_node = EchoNode()
            rclpy.spin(echo_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
