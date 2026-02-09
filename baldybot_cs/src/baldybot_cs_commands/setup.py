from setuptools import find_packages, setup

package_name = 'baldybot_cs_commands'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itti_jammy',
    maintainer_email='ittirath.dss@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_gripper_commands = baldybot_cs_commands.arm_gripper_commands:main',
            'slow_joy = baldybot_cs_commands.slow_joy:main',
            'two_wheel_joy = baldybot_cs_commands.two_wheel_joy:main',
        ],
    },
)
