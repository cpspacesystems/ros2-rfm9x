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
        super().__init__('minimal_subscriber')

        # Set up radio
        self.radio = radio

        self.subscription = self.create_subscription(
            Imu,
            'imu2',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.radio.send(bytes(str(msg.linear_acceleration.z),"utf-8"))
        self.get_logger().info('Z_accel: %s' % str(msg.linear_acceleration.z))


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
