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
                {'node_id': '40'},
                {'pin_cs': 'ground.CE0'},
                {'pin_rst': 'ground.D26'},
                {'topics': 'imu2/raw'},
                {'types': 'Imu'},
                {'pin_irq': 'ground.D6'}
            ]
        )
    ])
