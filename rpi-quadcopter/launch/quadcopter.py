
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pid',
            namespace='attitude/pitch_controller',
            executable='controller',
            name='pitch_controller',
            remappings=[
                ('state', '/imu/data/pitch'),
                ('control_effort', '/attitude/pitch_controller/control_effort')
            ],
            parameters=[
                {"Kp": 5.0},
                {"Kd": 0.0},
                {"Ki": 0.0},
                {"lower_limit": -250.0},
                {"upper_limit": 250.0},

            ]
        ),
        Node(
            package='pid',
            namespace='rate/pitch_controller',
            executable='controller',
            name='pitch_controller',
            remappings=[
                ('setpoint', '/attitude/pitch_controller/control_effort'),
                ('state', '/imu/data/gyro/pitch_rate')
            ],
            parameters=[
                {"Kp": 5.0},
                {"Kd": 0.0},
                {"Ki": 0.0}
            ]
        ),
        Node(
            package='quadcopter_motor_controller',
            executable='quadcopter_motor_controller_node',
            name='quadcopter_motor_controller',
            remappings=[
                ('pitch', '/rate/pitch_controller/control_effort')
            ],
            parameters=[
                {"Kp": 5.0},
                {"Kd": 0.0},
                {"Ki": 0.0}
            ]
        )

    ])