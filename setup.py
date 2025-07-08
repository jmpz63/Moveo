from setuptools import setup
import os
from glob import glob

package_name = 'arduino_arm_hardware'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][y]'))),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Install URDF files (if you put them here, otherwise in a separate robot_description package)
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf.xacro'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Custom ros2_control hardware interface for Arduino Mega RAMPS 1.4',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # No console scripts needed for a hardware interface plugin
        ],
        'hardware_interface.plugin': [
            'ArduinoArmHardware = arduino_arm_hardware.arduino_arm_hardware:ArduinoArmHardware',
        ],
    },
)
