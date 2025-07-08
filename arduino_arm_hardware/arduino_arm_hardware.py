import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from hardware_interface import SystemInterface, SystemCommandHandle, SystemStateHandle
from hardware_interface.return_codes import CallbackReturn, return_type

from controller_manager_msgs.srv import SetHardwareComponentState
from controller_manager_msgs.msg import HardwareComponentState

import serial
import time
import threading
import numpy as np

class ArduinoArmHardware(SystemInterface):
    """
    ROS 2 Control Hardware Interface for Arduino Mega + RAMPS 1.4 via serial.
    """

    def __init__(self):
        super().__init__()
        self._node = None
        self._serial_port = None
        self._baud_rate = None
        self._num_joints = 0
        self._steps_per_unit = [] # Conversion factor from ROS unit (radians) to Arduino steps

        self._hw_states = [] # Current joint positions (from Arduino, in ROS units)
        self._hw_commands = [] # Desired joint positions (from controller, in ROS units)

        self._serial_connection = None
        self._serial_lock = threading.Lock()
        self._serial_read_thread = None
        self._stop_thread = False
        self._latest_arduino_steps = [] # Raw steps received from Arduino

    def on_init(self, hardware_info: SystemInterface.HardwareInfo) -> CallbackReturn:
        """
        Initialize the hardware interface.
        """
        self._node = rclpy.create_node('arduino_arm_hardware_node')
        self._node.get_logger().info("Initializing ArduinoArmHardware...")

        self._serial_port = hardware_info.hardware_parameters['serial_port']
        self._baud_rate = int(hardware_info.hardware_parameters['baud_rate'])
        self._num_joints = int(hardware_info.hardware_parameters['num_joints'])

        # Get steps_per_unit for each joint
        for i in range(self._num_joints):
            param_name = f"steps_per_unit_{i}"
            if param_name in hardware_info.hardware_parameters:
                self._steps_per_unit.append(float(hardware_info.hardware_parameters[param_name]))
            else:
                self._node.get_logger().error(f"Missing parameter '{param_name}' for joint {i}.")
                return CallbackReturn.ERROR
        
        if len(self._steps_per_unit) != self._num_joints:
            self._node.get_logger().error(f"Mismatch in _steps_per_unit array size ({len(self._steps_per_unit)}) and num_joints ({self._num_joints}).")
            return CallbackReturn.ERROR

        self._hw_states = [0.0] * self._num_joints
        self._hw_commands = [0.0] * self._num_joints
        self._latest_arduino_steps = [0] * self._num_joints # Initialize raw steps

        # Validate joint configuration
        if len(hardware_info.joints) != self._num_joints:
            self._node.get_logger().error(f"Number of joints in URDF ({len(hardware_info.joints)}) does not match 'num_joints' parameter ({self._num_joints}).")
            return CallbackReturn.ERROR

        for joint in hardware_info.joints:
            if len(joint.command_interfaces) != 1 or joint.command_interfaces[0].name != "position":
                self._node.get_logger().error(f"Joint '{joint.name}' must have exactly one 'position' command interface.")
                return CallbackReturn.ERROR
            if len(joint.state_interfaces) != 1 or joint.state_interfaces[0].name != "position":
                self._node.get_logger().error(f"Joint '{joint.name}' must have exactly one 'position' state interface.")
                return CallbackReturn.ERROR

        self._node.get_logger().info("ArduinoArmHardware initialized successfully.")
        return CallbackReturn.SUCCESS

    def on_configure(self, previous_state: rclpy.lifecycle.State) -> CallbackReturn:
        """
        Configure the hardware interface (open serial).
        """
        self._node.get_logger().info("Configuring ArduinoArmHardware...")
        try:
            self._serial_connection = serial.Serial(self._serial_port, self._baud_rate, timeout=0.1)
            self._node.get_logger().info(f"Successfully opened serial port {self._serial_port} at {self._baud_rate} baud.")
            time.sleep(2) # Give Arduino time to reset after serial connection
            self._serial_connection.flushInput() # Clear any old data
            self._serial_connection.flushOutput()

            # Start serial read thread
            self._stop_thread = False
            self._serial_read_thread = threading.Thread(target=self._read_serial_data)
            self._serial_read_thread.daemon = True # Allow main program to exit even if thread is running
            self._serial_read_thread.start()

        except serial.SerialException as e:
            self._node.get_logger().error(f"Failed to open serial port {self._serial_port}: {e}")
            return CallbackReturn.ERROR

        self._node.get_logger().info("ArduinoArmHardware configured successfully.")
        return CallbackReturn.SUCCESS

    def on_cleanup(self, previous_state: rclpy.lifecycle.State) -> CallbackReturn:
        """
        Clean up the hardware interface (close serial).
        """
        self._node.get_logger().info("Cleaning up ArduinoArmHardware...")
        self._stop_thread = True
        if self._serial_read_thread and self._serial_read_thread.is_alive():
            self._serial_read_thread.join(timeout=1.0) # Wait for thread to finish
        if self._serial_connection and self._serial_connection.is_open:
            self._serial_connection.close()
            self._node.get_logger().info("Serial port closed.")
        self._node.destroy_node()
        self._node = None
        self._node.get_logger().info("ArduinoArmHardware cleaned up successfully.")
        return CallbackReturn.SUCCESS

    def on_activate(self, previous_state: rclpy.lifecycle.State) -> CallbackReturn:
        """
        Activate the hardware interface.
        """
        self._node.get_logger().info("Activating ArduinoArmHardware...")
        # Reset states and commands on activation
        self._hw_states = [0.0] * self._num_joints
        self._hw_commands = [0.0] * self._num_joints
        self._latest_arduino_steps = [0] * self._num_joints
        self._node.get_logger().info("ArduinoArmHardware activated successfully.")
        return CallbackReturn.SUCCESS

    def on_deactivate(self, previous_state: rclpy.lifecycle.State) -> CallbackReturn:
        """
        Deactivate the hardware interface.
        """
        self._node.get_logger().info("Deactivating ArduinoArmHardware...")
        self._node.get_logger().info("ArduinoArmHardware deactivated successfully.")
        return CallbackReturn.SUCCESS

    def on_error(self, previous_state: rclpy.lifecycle.State) -> CallbackReturn:
        """
        Handle errors.
        """
        self._node.get_logger().error("ArduinoArmHardware encountered an error!")
        return CallbackReturn.ERROR

    def on_shutdown(self, previous_state: rclpy.lifecycle.State) -> CallbackReturn:
        """
        Shutdown the hardware interface.
        """
        self._node.get_logger().info("Shutting down ArduinoArmHardware...")
        self._node.get_logger().info("ArduinoArmHardware shut down successfully.")
        return CallbackReturn.SUCCESS

    def export_state_interfaces(self):
        """
        Export state interfaces (what the hardware provides).
        """
        state_interfaces = []
        for i, joint_name in enumerate(self.info.joints):
            state_interfaces.append(SystemStateHandle(joint_name.name, "position", self._hw_states[i]))
        return state_interfaces

    def export_command_interfaces(self):
        """
        Export command interfaces (what the hardware accepts).
        """
        command_interfaces = []
        for i, joint_name in enumerate(self.info.joints):
            command_interfaces.append(SystemCommandHandle(joint_name.name, "position", self._hw_commands[i]))
        return command_interfaces

    def read(self, time_point: rclpy.time.Time, period: rclpy.duration.Duration) -> return_type:
        """
        Read the current state from the hardware.
        """
        with self._serial_lock:
            # Convert raw steps from Arduino to ROS units (e.g., radians)
            for i in range(self._num_joints):
                # Ensure _latest_arduino_steps has data for this joint
                if i < len(self._latest_arduino_steps):
                    self._hw_states[i] = self._latest_arduino_steps[i] / self._steps_per_unit[i]
                else:
                    self._node.get_logger().warn(f"No step data for joint {i} yet. Setting state to 0.")
                    self._hw_states[i] = 0.0
        return return_type.OK

    def write(self, time_point: rclpy.time.Time, period: rclpy.duration.Duration) -> return_type:
        """
        Write commands to the hardware.
        """
        with self._serial_lock:
            if not self._serial_connection or not self._serial_connection.is_open:
                self._node.get_logger().error("Serial port not open for writing.")
                return return_type.ERROR

            for i in range(self._num_joints):
                # Convert ROS unit command (e.g., radians) to Arduino steps
                target_steps = int(self._hw_commands[i] * self._steps_per_unit[i])
                command_str = f"J{i}:{target_steps}\n"
                try:
                    self._serial_connection.write(command_str.encode('utf-8'))
                    # self._node.get_logger().info(f"Sent: {command_str.strip()}") # For debugging
                except serial.SerialException as e:
                    self._node.get_logger().error(f"Failed to write to serial port: {e}")
                    return return_type.ERROR
        return return_type.OK

    def _read_serial_data(self):
        """
        Thread function to continuously read data from the serial port.
        Expected format: "S<joint0_steps>,<joint1_steps>,<joint2_steps>,..."
        """
        while not self._stop_thread:
            try:
                if self._serial_connection and self._serial_connection.is_open:
                    line = self._serial_connection.readline().decode('utf-8').strip()
                    if line.startswith("S"):
                        parts = line[1:].split(',')
                        if len(parts) == self._num_joints:
                            with self._serial_lock:
                                try:
                                    self._latest_arduino_steps = [int(p) for p in parts]
                                    # self._node.get_logger().info(f"Received steps: {self._latest_arduino_steps}") # For debugging
                                except ValueError:
                                    self._node.get_logger().warn(f"Could not parse serial data: {line}")
                        else:
                            self._node.get_logger().warn(f"Received incorrect number of joint states: {line}")
                    elif line: # Log other messages from Arduino
                        self._node.get_logger().debug(f"Arduino: {line}")
            except serial.SerialException as e:
                if not self._stop_thread: # Only log if not intentionally stopping
                    self._node.get_logger().error(f"Serial read error: {e}")
                break # Exit thread on serial error
            except UnicodeDecodeError:
                self._node.get_logger().warn("Could not decode serial data (bad characters).")
            except Exception as e:
                self._node.get_logger().error(f"Unexpected error in serial read thread: {e}")
                break # Exit thread on unexpected error
            time.sleep(0.001) # Small delay to prevent busy-waiting

