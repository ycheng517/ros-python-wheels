import rclpy

from builtin_interfaces.msg import Duration


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("minimal_publisher")
    publisher = node.create_publisher(Duration, "topic", 10)

    msg = Duration()
    i = 0

    def timer_callback():
        nonlocal i
        msg.sec = i
        i += 1
        node.get_logger().info('Publishing: "%s"' % msg.sec)
        publisher.publish(msg)

    timer_period = 0.5  # seconds
    timer = node.create_timer(timer_period, timer_callback)

    rclpy.spin(node)

    # Destroy the timer attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
