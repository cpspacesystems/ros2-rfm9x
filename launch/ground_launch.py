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
                {'pin_cs': 'board.CE1'},
                {'pin_rst': 'board.D13'},
                {'topics': 'imu1/raw'},
                {'types': 'Imu'},
                {'pin_irq': '5'}
            ]
        )
    ])
