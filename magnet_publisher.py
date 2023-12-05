import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MagnetPublisher(Node):

    def __init__(self):
        super().__init__('magnet_publisher')
        self.publisher_ = self.create_publisher(Bool, 'toggle_magnets', 10)
        self.timer = self.create_timer(2, self.timer_callback)

    def timer_callback(self):
        #boolean user input for magnet interrupts (yes or no)
        user_input = input("Turn magnets on?(y/n)? ")
        msg = Bool()
        msg.data = user_input.lower() == 'y'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    magnet_publisher = MagnetPublisher()
    rclpy.spin(magnet_publisher)

    magnet_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

