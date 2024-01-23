#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class MagnetClient(Node):
    def __init__(self):
        super().__init__('magnet_client')
        self.client = self.create_client(SetBool, 'toggle_magnets_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the server to come up...')

    def send_request(self, toggle):
        req = SetBool.Request()
        req.data = toggle
        return self.client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    magnet_client = MagnetClient()

    while True:
        user_input = input("Enter 'on' to turn on the magnet, 'off' to turn off, or 'exit' to quit: ")
        if user_input == 'exit':
            break
        toggle = user_input == 'on'

        magnet_client.get_logger().info('Requesting to toggle magnets: %s' % toggle)
        future = magnet_client.send_request(toggle)

        rclpy.spin_until_future_complete(magnet_client, future)
        try:
            response = future.result()
            magnet_client.get_logger().info('Service response: %s' % response.success)
        except Exception as e:
            magnet_client.get_logger().error('Service call failed: %s' % str(e))

    magnet_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
