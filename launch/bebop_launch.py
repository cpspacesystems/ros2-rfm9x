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
#                {'node_id': '42'},
#                {'pin_cs': 'board.CE1'},
#                {'pin_rst': 'board.D13'},
#                {'topics': 'imu3;imu2;imu1'},
#                {'props': '"X="+str(msg.linear_acceleration.x)+", Y="+str(msg.linear_acceleration.y)+", Z="+str(msg.linear_acceleration.z);"X="+str(msg.linear_acceleration.x)+", Y="+str(msg.linear_acceleration.y)+", Z="+str(msg.linear_acceleration.z);str(msg.linear_acceleration.z)'},
#                {'labels': 'LimAcc;LinAcc;Zero'},
#                {'types': 'Imu;Imu;Imu'}

                {'node_id': '42'},
                {'pin_cs': 'board.CE1'},
                {'pin_rst': 'board.D13'},
                {'topics': 'imu0/raw;imu1/raw;imu2/raw'},
                {'types': 'Imu;Imu;Imu'}
            ]
        )
    ])
