import rclpy
from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from hardware_interface import SystemInterface, SystemCommandHandle, SystemStateHandle # Re-enabled SystemCommandHandle, SystemStateHandle
from hardware_interface.return_codes import CallbackReturn, return_type # Re-enabled return_type

# from controller_manager_msgs.srv import SetHardwareComponentState
# from controller_manager_msgs.msg import HardwareComponentState

# import serial
# import time
# import threading
# import numpy as np

# ADDED FOR DEBUGGING: This print will show if the Python file is even being loaded
print("--- ArduinoArmHardware.py: Starting module import ---")

class ArduinoArmHardware(SystemInterface):
    """
    Minimal ROS 2 Control Hardware Interface for debugging plugin loading.
    """

    def __init__(self):
        super().__init__()
        # Added for debugging: Print statement in constructor
        print("--- ArduinoArmHardware: __init__ called ---")
        self._node = None # Will be created in on_init
        self._num_joints = 0 # Keep for on_init validation

    def on_init(self, hardware_info: SystemInterface.HardwareInfo) -> CallbackReturn:
        """
        Initialize the hardware interface (minimal version).
        """
        # RE-ENABLED: rclpy node creation and logging for debugging
        try:
            rclpy.init(args=None) # Initialize rclpy if not already initialized
            self._node = Node('arduino_arm_hardware_node')
            self._node.get_logger().info("--- ArduinoArmHardware: on_init called, node created. ---")
            self._node.get_logger().info(f"--- Hardware Info: {hardware_info} ---")

            # Minimal parameter parsing to prevent crash if not found
            self._num_joints = int(hardware_info.hardware_parameters.get('num_joints', 0))
            self._node.get_logger().info(f"--- Parsed num_joints: {self._num_joints} ---")

            # Validate joint configuration (minimal check)
            if len(hardware_info.joints) != self._num_joints:
                self._node.get_logger().error(f"--- ERROR: Mismatch in num_joints. URDF: {len(hardware_info.joints)}, Param: {self._num_joints} ---")
                return CallbackReturn.ERROR

            self._node.get_logger().info("--- ArduinoArmHardware initialized (minimal) successfully. ---")
            return CallbackReturn.SUCCESS
        except Exception as e:
            # Catch any exception during on_init and print it
            print(f"--- ERROR in ArduinoArmHardware.on_init: {e} ---")
            return CallbackReturn.ERROR

    def on_configure(self, previous_state: rclpy.lifecycle.State) -> CallbackReturn:
        if self._node:
            self._node.get_logger().info("--- Configuring ArduinoArmHardware (minimal)... ---")
        return CallbackReturn.SUCCESS

    def on_cleanup(self, previous_state: rclpy.lifecycle.State) -> CallbackReturn:
        if self._node:
            self._node.get_logger().info("--- Cleaning up ArduinoArmHardware (minimal)... ---")
            self._node.destroy_node()
            self._node = None
        return CallbackReturn.SUCCESS

    def on_activate(self, previous_state: rclpy.lifecycle.State) -> CallbackReturn:
        if self._node:
            self._node.get_logger().info("--- Activating ArduinoArmHardware (minimal)... ---")
        return CallbackReturn.SUCCESS

    def on_deactivate(self, previous_state: rclpy.lifecycle.State) -> CallbackReturn:
        if self._node:
            self._node.get_logger().info("--- Deactivating ArduinoArmHardware (minimal)... ---")
        return CallbackReturn.SUCCESS

    def on_error(self, previous_state: rclpy.lifecycle.State) -> CallbackReturn:
        if self._node:
            self._node.get_logger().error("--- ArduinoArmHardware encountered an error (minimal)! ---")
        return CallbackReturn.ERROR

    def on_shutdown(self, previous_state: rclpy.lifecycle.State) -> CallbackReturn:
        if self._node:
            self._node.get_logger().info("--- Shutting down ArduinoArmHardware (minimal)... ---")
            # rclpy.shutdown() # Only shutdown if rclpy.init was called without args
        return CallbackReturn.SUCCESS

    def export_state_interfaces(self):
        # Return empty list for minimal test
        return []

    def export_command_interfaces(self):
        # Return empty list for minimal test
        return []

    def read(self, time_point: rclpy.time.Time, period: rclpy.duration.Duration) -> return_type:
        return return_type.OK

    def write(self, time_point: rclpy.time.Time, period: rclpy.duration.Duration) -> return_type:
        return return_type.OK

    # Removed serial reading thread and related methods
    # def _read_serial_data(self):
    #     pass
