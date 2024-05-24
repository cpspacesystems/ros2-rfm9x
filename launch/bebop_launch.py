from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rfm9x',
            executable='radio',
            name='bebop_radio',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'lora_topics_sub': '/imu1/raw'},
                {'lora_topics_pub': ''},
                {'message_types_sub': 'sensor_msgs/Imu'},
                {'message_types_pub': ''},
                {'spi_sck': 'SCK'},
                {'spi_mosi': 'MOSI'},
                {'spi_miso': 'MISO'},
                {'spi_cs': 'CE1'},
                {'spi_reset': 'D13'}
            ]
        )
    ])
