import gzip
import rclpy
from rclpy.node import Node
import board
import busio
import digitalio
import adafruit_rfm9x
import importlib

from rclpy.serialization import serialize_message, deserialize_message

class Radio(Node):

    def __init__(self):
        super().__init__('lora_bridge_node')

        # Declare parameters
        self.declare_parameter('lora_topics_sub', '')
        self.declare_parameter('lora_topics_pub', '')
        self.declare_parameter('message_types_sub', '')
        self.declare_parameter('message_types_pub', '')
        self.declare_parameter('spi_sck', 'SCK')
        self.declare_parameter('spi_mosi', 'MOSI')
        self.declare_parameter('spi_miso', 'MISO')
        self.declare_parameter('spi_cs', 'D5')
        self.declare_parameter('spi_reset', 'D25')

        # Get parameters
        self.lora_topics_sub = list(filter(None, self.get_parameter('lora_topics_sub').get_parameter_value().string_value.split(',')))
        self.lora_topics_pub = list(filter(None, self.get_parameter('lora_topics_pub').get_parameter_value().string_value.split(',')))
        self.message_types_sub = list(filter(None, self.get_parameter('message_types_sub').get_parameter_value().string_value.split(',')))
        self.message_types_pub = list(filter(None, self.get_parameter('message_types_pub').get_parameter_value().string_value.split(',')))

        self.spi_sck = self.get_parameter('spi_sck').get_parameter_value().string_value
        self.spi_mosi = self.get_parameter('spi_mosi').get_parameter_value().string_value
        self.spi_miso = self.get_parameter('spi_miso').get_parameter_value().string_value
        self.spi_cs = self.get_parameter('spi_cs').get_parameter_value().string_value
        self.spi_reset = self.get_parameter('spi_reset').get_parameter_value().string_value

        # Log received parameters
        self.get_logger().info(f"lora_topics_sub: {self.lora_topics_sub}")
        self.get_logger().info(f"lora_topics_pub: {self.lora_topics_pub}")
        self.get_logger().info(f"message_types_sub: {self.message_types_sub}")
        self.get_logger().info(f"message_types_pub: {self.message_types_pub}")

        self.get_logger().info(f"spi_sck: {self.spi_sck}")
        self.get_logger().info(f"spi_mosi: {self.spi_mosi}")
        self.get_logger().info(f"spi_miso: {self.spi_miso}")
        self.get_logger().info(f"spi_cs: {self.spi_cs}")
        self.get_logger().info(f"spi_reset: {self.spi_reset}")

        # Create subscription and publisher for each topic with appropriate message type
        self.subs = []
        self.pubs = []
        self.sub_msg_classes = []
        self.pub_msg_classes = []

        for i, topic in enumerate(self.lora_topics_sub):
            package_name, message_name = self.message_types_sub[i].split('/')
            msg_module = importlib.import_module(f"{package_name}.msg")
            msg_class = getattr(msg_module, message_name)
            subscription = self.create_subscription(msg_class, topic, self.lora_receive_callback, 10)
            self.subs.append(subscription)
            self.sub_msg_classes.append(msg_class)

        for i, topic in enumerate(self.lora_topics_pub):
            package_name, message_name = self.message_types_pub[i].split('/')
            msg_module = importlib.import_module(f"{package_name}.msg")
            msg_class = getattr(msg_module, message_name)
            publisher = self.create_publisher(msg_class, topic, 10)
            self.pubs.append(publisher)
            self.pub_msg_classes.append(msg_class)

        # Initialize SPI and LoRa using parameters for GPIO pins
        self.spi = busio.SPI(getattr(board, self.spi_sck), MOSI=getattr(board, self.spi_mosi), MISO=getattr(board, self.spi_miso))
        self.cs = digitalio.DigitalInOut(getattr(board, self.spi_cs))
        self.reset = digitalio.DigitalInOut(getattr(board, self.spi_reset))
        self.rfm9x = adafruit_rfm9x.RFM9x(self.spi, self.cs, self.reset, 915.0)

        # Create timer for periodic receiving
        self.timer = self.create_timer(1.0, self.lora_send_callback)

    def lora_receive_callback(self, msg):
        self.get_logger().info(f"Received message to send over LoRa: {msg}")
        serialized_msg = serialize_message(msg)

        serialized_msg = gzip.compress(serialized_msg)

        if serialized_msg:
            self.rfm9x.send(serialized_msg)

    def lora_send_callback(self):
        packet = self.rfm9x.receive()
        if packet is not None:
            self.get_logger().info(f"Received message from LoRa")
            packet = gzip.decompress(packet)
            for msg_class, publisher in zip(self.pub_msg_classes, self.pubs):
                deserialized_msg = deserialize_message(packet, msg_class)
                if deserialized_msg:
                    publisher.publish(deserialized_msg)

    def serialize_message(self, msg):
        try:
            return msg.serialize()
        except Exception as e:
            self.get_logger().error(f"Serialization error: {e}")
            return None

    def deserialize_message(self, data, msg_class):
        try:
            msg = msg_class()
            msg.deserialize(data)
            return msg
        except Exception as e:
            self.get_logger().error(f"Deserialization error: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = Radio()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
