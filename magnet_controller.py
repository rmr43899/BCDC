#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

class MagnetController:

    def __init__(self):
        rospy.init_node('magnet_controller', anonymous=True)

        # In ROS1, parameters are retrieved slightly differently
        if rospy.has_param('~relay_pins'):
            self.relay_pins = rospy.get_param('~relay_pins')
        else:
            self.relay_pins = [26]

        GPIO.setmode(GPIO.BCM)
        for pin in self.relay_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)  # Initially, keep magnets ON

        # Log message indicating initial state of relays
        rospy.loginfo('Initial state: Magnets are OFF.')

        self.subscription = rospy.Subscriber(
            'toggle_magnets',
            Bool,
            self.listener_callback)

    def listener_callback(self, msg):
        if msg.data:  # If True
            rospy.loginfo('Turning OFF magnets for take-off')
            for pin in self.relay_pins:
                GPIO.output(pin, GPIO.HIGH)
        else:  # If False
            rospy.loginfo('Turning ON magnets')
            for pin in self.relay_pins:
                GPIO.output(pin, GPIO.LOW)

    def cleanup(self):
        rospy.loginfo('Terminating node. Turning OFF magnets.')
        for pin in self.relay_pins:
            GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup()

if __name__ == '__main__':
    magnet_controller = MagnetController()
    rospy.on_shutdown(magnet_controller.cleanup)
    rospy.spin()
