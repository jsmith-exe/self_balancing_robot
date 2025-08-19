from setuptools import setup

package_name = 'esp32'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # this is src/esp32/esp32
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/esp32']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ESP32 tools and bridges',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # executable name  ->  module:function
            'imu_bridge = esp32.serial_to_ros2:main',
            # (optional compatibility alias; remove if you don’t want it)
            'esp32_imu_serial_bridge = esp32.serial_to_ros2:main',
        ],
    },
)
