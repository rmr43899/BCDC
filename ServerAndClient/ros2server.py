#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from MockRPi import GPIO
from std_srvs.srv import SetBool
import sys
sys.path.append('/home/rmr4389/ros2_ws/src/magnet_control/magnet_control')

class MagnetServer(Node):
    def __init__(self):
        super().__init__('magnet_server')
        self.service = self.create_service(SetBool, 'toggle_magnets_service', self.handle_toggle_request)
        self.relay_pin = 26
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT)

    def handle_toggle_request(self, request, response):
        try:
            self.get_logger().info('Received request to toggle magnets: %s' % request.data)
            GPIO.output(self.relay_pin, GPIO.LOW if request.data else GPIO.HIGH)
            response.success = True
            return response
        except Exception as e:
            self.get_logger().error('Error in handling request: %s' % str(e))
            response.success = False
            return response

def main(args=None):
    rclpy.init(args=args)
    magnet_server = MagnetServer()
    rclpy.spin(magnet_server)
    magnet_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
