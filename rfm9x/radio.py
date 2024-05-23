import gzip
import time
import busio
from digitalio import DigitalInOut
import gpiozero as io
import board
import adafruit_rfm9x

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message

#import sensor_msgs
#import geometry_msgs
#import std_msgs
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

class Radio(Node):
    def __init__(self):
        super().__init__('radio')

        # Declare ROS2 parameters
        self.declare_parameter('topics', '')
        self.declare_parameter('types', '')
        self.declare_parameter('pin_cs', '')
        self.declare_parameter('pin_rst', '')
        self.declare_parameter('pin_irq', '')
        self.declare_parameter('node_id', '')

        # Set up radio
        pin_cs = self.get_parameter('pin_cs').get_parameter_value().string_value
        pin_rst = self.get_parameter('pin_rst').get_parameter_value().string_value
        node_id = int(self.get_parameter('node_id').get_parameter_value().string_value)

        self.radio = adafruit_rfm9x.RFM9x(spi, DigitalInOut(eval(pin_cs)), DigitalInOut(eval(pin_rst)), 915.0)
        self.radio.tx_power = 23
        self.node = node_id

        # Get list of topics
        self.topics = self.get_parameter('topics').get_parameter_value().string_value.split(';')
        self.types = self.get_parameter('types').get_parameter_value().string_value.split(';')

        self.get_logger().info("INIT: %s" % [i for i in zip(self.topics, self.types)])

        # Check if we are receiving or sending
        pin_irq = self.get_parameter('pin_irq').get_parameter_value().string_value
        if pin_irq != "":
            # Set up LoRa Message Interrupt
            pin_irq = int(pin_irq)
            self.irq = io.Button(pin_irq)
            self.irq.when_pressed = self.on_receive
            self.packet_received = False

            self.timer = self.create_timer(0.02, self.loop)
        else:
            # Set up ROS Subscribers
            self.subscribers = [self.create_subscription(
                eval(type),
                topic,
                self.build_callback(topic, type),
                10) for (topic, type) in zip(self.topics, self.types)]


    def loop(self):
        if self.packet_received:
            print('received message!')
            self.packet_received = False

    def on_receive(self, rfm9x_irq):
        print('irq detected ', rfm9x_irq, self.radio.rx_done)
        if self.radio.rx_done:
            packet = self.radio.receive(timeout=None)
            if packet is not None:
                self.packet_received = True
                self.process_received_packet(packet)

    def process_received_packet(self, packet):
        # Deserialize packet
        try:
            # Assume the packet structure: [type_length, type_name, data]
            type_length = packet[0]
            type_name = packet[1:1+type_length].decode('utf-8')
            data = packet[1+type_length:]

            data = gzip.decompress(data)

            msg_type = eval(type_name)  # Dynamically get message type
            msg = msg_type()
            deserialize_message(data, msg)

            # Publish the deserialized message to the appropriate topic
            topic = next((t for t, l in zip(self.topics, self.types) if l == type_name), None)
            if topic:
                pub = self.create_publisher(msg_type, topic, 10)
                pub.publish(msg)
                self.get_logger().info(f"Received and published {type_name} message on {topic}")

        except Exception as e:
            self.get_logger().error(f"Failed to process received packet: {e}")

    def build_callback(self, topic, type):
        return (lambda msg: self.listener_callback(msg, topic, type))

    def listener_callback(self, msg, topic, type):
        # Serialize the message
        try:
            msg_type = eval(type)  # Dynamically get message type
            data = serialize_message(msg)
            data = gzip.compress(data)
            type_name = type.encode('utf-8')
            packet = bytes([len(type_name)]) + type_name + data
            self.radio.send(packet, keep_listening=True)
            self.get_logger().info(f"Sent {type} [{topic}] message")
        except Exception as e:
            self.get_logger().error(f"Failed to serialize and send message: {e}")

def main(args=None):
    try:
        rclpy.init(args=args)
        radio_node = Radio()
        rclpy.spin(radio_node)
        radio_node.destroy_node()
        rclpy.shutdown()

    except RuntimeError as error:
        print('RFM9x Error: ', error)

if __name__ == '__main__':
    main()
