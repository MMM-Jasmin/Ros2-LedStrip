import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class LedStrip(Node):

    def __init__(self):
        super().__init__('led_strip')
        self.subscription = self.create_subscription(
            String,
            'led_strip',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    led_strip = LedStrip()

    try:
        rclpy.spin(led_strip)
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except BaseException:
        print('exception in server:', file=sys.stderr)
        raise
    # finally:
    #     Destroy the node explicitly
    #     (optional - Done automatically when node is garbage collected)
    #     led_strip.destroy_node()
    #     rclpy.shutdown()

if __name__ == '__main__':
    main()
