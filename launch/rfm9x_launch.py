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
                {'topics': 'imu2;imu1'},
                {'props': '"X="+str(msg.linear_acceleration.x)+", Y="+str(msg.linear_acceleration.y)+", Z="+str(msg.linear_acceleration.z);str(msg.linear_acceleration.z)'},
                {'labels': 'LinAcc;Zero'},
                {'types': 'Imu;Imu'}
            ]
        )
    ])