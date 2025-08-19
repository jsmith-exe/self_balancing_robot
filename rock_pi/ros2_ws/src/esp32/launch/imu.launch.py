from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyESP32')
    baud_arg = DeclareLaunchArgument('baud', default_value='115200')

    node = Node(
        package='esp32',
        executable='imu_bridge',   # <-- matches setup.py console_scripts
        name='esp32_imu_serial_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
        }],
    )
    return LaunchDescription([port_arg, baud_arg, node])
