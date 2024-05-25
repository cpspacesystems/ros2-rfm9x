from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rfm9x',
            executable='radio',
            name='ground_radio',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'lora_topics_pub': '/imu1/raw'},
                {'lora_topics_sub': ''},
                {'message_types_pub': 'sensor_msgs/Imu'},
                {'message_types_sub': ''},
                {'spi_sck': 'SCK'},
                {'spi_mosi': 'MOSI'},
                {'spi_miso': 'MISO'},
                {'spi_cs': 'CE1'},
                {'spi_reset': 'D27'},
                {'irq_pin': '17'}
            ]
        )
    ])
