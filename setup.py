from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'arduino_arm_hardware'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][y]'))),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf.xacro'))),
        # Install the plugin XML file itself
        (os.path.join('share', package_name), glob('arduino_arm_hardware_plugins.xml')),
        # NEW CRITICAL ADDITION: Register plugin with ament_index
        # This creates a file named 'arduino_arm_hardware' in
        # share/ament_index/resource_index/pluginlib/
        # The content of this file is the relative path to the actual plugin XML
        (os.path.join('share', 'ament_index', 'resource_index', 'pluginlib'),
            [os.path.join('resource', 'arduino_arm_hardware')]), # This resource file needs to exist in your source package
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
