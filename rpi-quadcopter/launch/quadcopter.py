
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_setpoint_node(name, setpoint_value):
   return Node(package='rpi-quadcopter', executable='setpoint_node', name=name,
        remappings=[
            ('setpoint', '/' + name)
        ],
        parameters=[
            {"setpoint_value": setpoint_value}
        ])

def generate_pid_controller(namespace, base_name, Kp, Kd, Ki, lower_limit, upper_limit):
    return Node(
        package='pid',
        namespace=namespace + '/' + base_name + '_controller',
        executable='controller',
        name=base_name+ '_controller',
        remappings=[
            ('setpoint', '/desired_' + base_name),
            ('state', '/' + base_name),
            ('control_effort', '/' + namespace + '/' + base_name + '_controller/control_effort')
        ],
        parameters=[
            {"Kp": Kp},
            {"Kd": Kd},
            {"Ki": Ki},
            {"lower_limit": lower_limit},
            {"upper_limit": upper_limit}
        ])

def generate_rate_pid_controller(base_name, Kp, Kd, Ki, lower_limit, upper_limit):
    return Node(
        package='pid',
        namespace='rate/' + base_name + '_controller',
        executable='controller',
        name=base_name+ '_controller',
        remappings=[
            ('setpoint', '/attitude/' + base_name + '_controller/control_effort'),
            ('state', '/' + base_name + '_rate'),
            ('control_effort', '/rate' + '/' + base_name + '_controller/control_effort')
        ],
        parameters=[
            {"Kp": Kp},
            {"Kd": Kd},
            {"Ki": Ki},
            {"lower_limit": lower_limit},
            {"upper_limit": upper_limit}
        ])

def generate_launch_description():
    desired_roll = generate_setpoint_node('desired_roll', 0.0)
    desired_pitch = generate_setpoint_node('desired_pitch', 0.0)
    desired_yaw = generate_setpoint_node('desired_yaw', 0.0)
    desired_altitude = generate_setpoint_node('desired_altitude', 2.0)

    altitude_controller = generate_pid_controller('altitude', 'altitude', 5.0, 0.0, 0.0, 0.0, 250.0)

    imu_rpy = Node(package='imu_rpy', executable='imu_rpy_node', name='imu_rpy')

    roll_controller = generate_pid_controller('attitude', 'roll', 5.0, 0.0, 0.0, -250.0, 250.0)
    pitch_controller = generate_pid_controller('attitude', 'pitch', 5.0, 0.0, 0.0, -250.0, 250.0)
    yaw_controller = generate_pid_controller('attitude', 'yaw', 5.0, 0.0, 0.0, -250.0, 250.0)

    roll_rate_controller = generate_rate_pid_controller('roll', 5.0, 0.0, 0.0, -250.0, 250.0)
    pitch_rate_controller = generate_rate_pid_controller('pitch', 5.0, 0.0, 0.0, -250.0, 250.0)
    yaw_rate_controller = generate_rate_pid_controller('yaw', 5.0, 0.0, 0.0, -250.0, 250.0)

    quadcopter_motor_controller = Node(package='quadcopter_motor_controller', executable='quadcopter_motor_controller_node',
        name='quadcopter_motor_controller',
        remappings=[
            ('thrust', '/altitude/altitude_controller/control_effort'),
            ('roll', '/rate/roll_controller/control_effort'),
            ('pitch', '/rate/pitch_controller/control_effort'),
            ('yaw', '/rate/yaw_controller/control_effort')
        ],
        parameters=[
            {"motor_front_gpio": 5},
            {"motor_right_gpio": 6},
            {"motor_back_gpio": 12},
            {"motor_left_gpio": 13}
        ])


    return LaunchDescription([ desired_roll, desired_pitch, desired_yaw,
                               desired_altitude, altitude_controller, imu_rpy,
                               roll_controller, pitch_controller, yaw_controller,
                               roll_rate_controller, pitch_rate_controller, yaw_rate_controller,
                               quadcopter_motor_controller])