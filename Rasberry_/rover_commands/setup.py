from setuptools import find_packages, setup

package_name = 'rover_commands'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_launch.py']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xplore',
    maintainer_email='xplore@todo.todo',
    description='Package for controlling the rover and interfacing with a camera.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber = rover_commands.subscriber:main',
            'arduino_test_ros = rover_commands.arduino_test_ros:main',
            'motor_subscriber = rover_commands.motor_subscriber:main',
            'smartphone = rover_commands.smartphone:main',
            'rpi_velocity_subscriber = rover_commands.rpi_velocity_subscriber:main',
            'diff_drive = rover_commands.diff_drive:main',
            'rpi_velocity_I2C_subscriber = rover_commands.rpi_velocity_I2C_subscriber:main',
            'rpi_arm_motor_publisher = rover_commands.rpi_arm_motor_publisher:main',
            'velocity_2_wheels = rover_commands.velocity_2_wheels:main',
            'diff_drive_with_arm = rover_commands.diff_drive_with_arm:main',
            'two_axes_controller = rover_commands.two_axes_controller:main',
        ],
    },
)
