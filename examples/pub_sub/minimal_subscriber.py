import rclpy

from builtin_interfaces.msg import Duration


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("minimal_subscriber")

    node.create_subscription(
        Duration,
        "topic",
        lambda msg: node.get_logger().info('I heard: "%d"' % msg.sec),
        10,
    )

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
