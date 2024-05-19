import time
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
import adafruit_rfm9x

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

class MinimalSubscriber(Node):

    def __init__(self, radio):
        super().__init__('radio')

        self.declare_parameter('topics', '')
        self.declare_parameter('props', '')
        self.declare_parameter('labels', '')
        self.declare_parameter('types', '')

        # Set up radio
        self.radio = radio

        self.topics = self.get_parameter('topics').get_parameter_value().string_value.split(';')
        self.props = self.get_parameter('props').get_parameter_value().string_value.split(';')
        self.labels = self.get_parameter('labels').get_parameter_value().string_value.split(';')
        self.types = self.get_parameter('types').get_parameter_value().string_value.split(';')

        self.subscribers = [self.create_subscription(
            eval(type),
            topic,
            self.build_callback(topic, prop, label),
            10) for (topic, prop, label, type) in zip(self.topics, self.props, self.labels, self.types)]

    def build_callback(self, topic, prop, label):
        return (lambda msg: self.listener_callback(msg, topic, prop, label))

    def listener_callback(self, msg, topic, prop, label):
        self.radio.send(bytes(label + ' [' + topic + ']: ' + eval(prop),"utf-8"))
        self.get_logger().info(label + ' [' + topic + ']: %s' % eval(prop))


def main(args=None):
    try:
        # Initialize the radio
        rfm = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
        rfm.tx_power = 23

        # Create the node and register it woth ROS
        rclpy.init(args=args)
        minimal_subscriber = MinimalSubscriber(rfm)
        rclpy.spin(minimal_subscriber)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

    except RuntimeError as error:
        # Thrown on version mismatch
        print('RFM9x Error: ', error)


if __name__ == '__main__':
    main()
