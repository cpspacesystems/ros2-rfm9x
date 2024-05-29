import gzip
import rclpy
from rclpy.node import Node
from gpiozero import Button
import board
import busio
import digitalio
import adafruit_rfm9x
import importlib

from rclpy.serialization import serialize_message, deserialize_message

class Radio(Node):

    def __init__(self):
        super().__init__('radio')

        # Declare parameters
        self.declare_parameter('node_id', 255)
        self.declare_parameter('dest_id', 255)
        self.declare_parameter('tx_pwr', 13)
        self.declare_parameter('topics_sub', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('topics_pub', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('msg_types_sub', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('msg_types_pub', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('spi_sck', 'SCK')
        self.declare_parameter('spi_mosi', 'MOSI')
        self.declare_parameter('spi_miso', 'MISO')
        self.declare_parameter('spi_cs', 'D5')
        self.declare_parameter('spi_reset', 'D25')
        self.declare_parameter('irq_pin', '5')

        # Get parameters
        self.node_id = self.get_parameter('node_id').get_parameter_value().integer_value
        self.dest_id = self.get_parameter('dest_id').get_parameter_value().integer_value
        self.tx_pwr = self.get_parameter('tx_pwr').get_parameter_value().integer_value
        self.lora_topics_sub = self.get_parameter('topics_sub').get_parameter_value().string_array_value
        self.lora_topics_pub = self.get_parameter('topics_pub').get_parameter_value().string_array_value
        self.message_types_sub = self.get_parameter('msg_types_sub').get_parameter_value().string_array_value
        self.message_types_pub = self.get_parameter('msg_types_pub').get_parameter_value().string_array_value
        self.spi_sck = self.get_parameter('spi_sck').get_parameter_value().string_value
        self.spi_mosi = self.get_parameter('spi_mosi').get_parameter_value().string_value
        self.spi_miso = self.get_parameter('spi_miso').get_parameter_value().string_value
        self.spi_cs = self.get_parameter('spi_cs').get_parameter_value().string_value
        self.spi_reset = self.get_parameter('spi_reset').get_parameter_value().string_value
        self.interrupt_pin = int(self.get_parameter('irq_pin').get_parameter_value().string_value)

        # Initialize SPI and LoRa using parameters for GPIO pins
        self.spi = busio.SPI(getattr(board, self.spi_sck), MOSI=getattr(board, self.spi_mosi), MISO=getattr(board, self.spi_miso))
        self.cs = digitalio.DigitalInOut(getattr(board, self.spi_cs))
        self.reset = digitalio.DigitalInOut(getattr(board, self.spi_reset))
        self.rfm9x = adafruit_rfm9x.RFM9x(self.spi, self.cs, self.reset, 915.0)
        self.rfm9x.tx_power = self.tx_pwr
        self.rfm9x.node = self.node_id
        self.rfm9x.destination = self.dest_id


        self.get_logger().info(f"Initializing radio (SPI:[CK:{self.spi_sck} MO:{self.spi_mosi} MI:{self.spi_miso}] CS:{self.spi_cs} RST:{self.spi_reset} PWR:{self.tx_pwr} NODE:{self.node_id} DST:{self.dest_id}")

        # Create subscription and publisher for each topic with appropriate message type
        self.subs = {}
        self.pubs = {}
        self.topic_type_map = {}

        for i, topic in enumerate(self.lora_topics_sub):
            package_name, message_name = self.message_types_sub[i].split('/')
            msg_module = importlib.import_module(f"{package_name}.msg")
            msg_class = getattr(msg_module, message_name)
            subscription = self.create_subscription(msg_class, topic, (lambda msg, t=topic: self.sub_callback(msg, t)), 10)
            self.subs[topic] = subscription

        if len(self.lora_topics_sub) > 0:
            self.get_logger().info("Subscribing and sending the following topics: %s" % self.lora_topics_sub)

        for j, ptopic in enumerate(self.lora_topics_pub):
            package_name, message_name = self.message_types_pub[j].split('/')
            msg_module = importlib.import_module(f"{package_name}.msg")
            msg_class = getattr(msg_module, message_name)
            self.topic_type_map[ptopic] = msg_class
            publisher = self.create_publisher(msg_class, ptopic, 10)
            self.pubs[ptopic] = publisher

        if len(self.lora_topics_pub) > 0:
            self.get_logger().info("Receiving and publishing the following topics: %s" % list(self.topic_type_map.keys()))

        # Create timer for periodic receiving
        self.timer = self.create_timer(0.1, self.lora_rx_callback)

        self.get_logger().info("Done Initializing!")

    def sub_callback(self, msg, topic_name):
        # Serialize the message
        serialized_msg = serialize_message(msg)

        # Concatenate topic name and serialized message
        # Use a delimiter to separate the topic name and the serialized message
        delimiter = b'|'
        payload = topic_name.encode() + delimiter + serialized_msg

        # Compress the payload
        compressed_payload = gzip.compress(payload)

        if compressed_payload:
            self.get_logger().debug(f"Sending [{topic_name}]: {msg}")
            self.rfm9x.send(compressed_payload, keep_listening=True)

    def lora_rx_callback(self):
        if self.rfm9x.rx_done:
            packet = self.rfm9x.receive()
            if packet is not None:
                # Decompress the payload
                decompressed_payload = gzip.decompress(packet)

                # Split the payload into topic name and serialized message
                delimiter = b'|'
                topic_name, serialized_msg = decompressed_payload.split(delimiter, 1)

                # Decode the topic name
                topic_name = topic_name.decode()

                # Find the appropriate message type and publisher for the topic
                if topic_name in self.pubs:
                    msg_class = self.topic_type_map[topic_name]
                    deserialized_msg = deserialize_message(serialized_msg, msg_class)
                    publisher = self.pubs[topic_name]

                    if deserialized_msg:
                        self.get_logger().debug(f"Received [{topic_name}]: {deserialized_msg}")
                        publisher.publish(deserialized_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Radio()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
