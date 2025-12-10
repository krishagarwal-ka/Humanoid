import serial
import time
import struct
import threading
from enum import Enum
from typing import Optional, List, Dict, Any, Callable
import math

class MotorState(Enum):
    DISABLED = "disabled"
    ENABLED = "enabled"
    FAULT = "fault"
    HOMING = "homing"
    MOVING = "moving"
    HOLDING = "holding"

class ControlMode(Enum):
    CURRENT = "current"
    SPEED = "speed"
    POSITION = "position"
    HOMING = "homing"
    MIT = "mit"

class FaultType(Enum):
    NONE = "none"
    VOLTAGE = "voltage"
    CURRENT = "current"
    TEMPERATURE = "temperature"
    ENCODER = "encoder"
    HARDWARE = "hardware"
    SOFTWARE = "software"

class CANMotorController:
    """
    A user-friendly interface for controlling CAN-based motors.
    
    Example usage:
        # Basic usage with default address
        motor = CANMotorController("/dev/tty.usbserial-2130")
        motor.connect()
        
        # Usage with specific CAN address
        motor2 = CANMotorController("COM3", device_id=2)
        motor2.connect()
        
        # Move to specific angles
        motor.move_to(90)  # Move to 90 degrees
        motor.move_relative(45)  # Move 45 degrees from current position
        
        # Control modes
        motor.set_speed(100)  # 100 RPM
        motor.set_current(0.5)  # 0.5A torque
        
        # System commands
        motor.set_home()  # Set current position as home
        motor.go_home()   # Return to home position
        
        motor.disconnect()
    """
    
    def __init__(self, port: str, device_id: int, baudrate: int = 2000000):
        """
        Initialize the motor controller.
        
        Args:
            port: Serial port name (e.g., "/dev/ttyUSB0", "COM3", "/dev/tty.usbserial-2130")
            device_id: CAN device address (1-254). Default is 1.
            baudrate: Serial communication baudrate. Default is 2000000.
        """
        self.port = port
        self.device_id = device_id
        self.baudrate = baudrate
        self.serial_conn = None
        self.is_connected = False
        self.running = False
        self.callbacks = {
            'state_change': [],
            'fault': [],
            'position_update': [],
            'status_update': []
        }
        
        # Validate device ID
        if not (1 <= device_id <= 254):
            raise ValueError(f"Device ID must be between 1 and 254, got {device_id}")
        
        # Motor state
        self.state = MotorState.DISABLED
        self.control_mode = ControlMode.POSITION
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.current_speed = 0.0
        self.current_torque = 0.0
        self.voltage = 0.0
        self.temperature = 0.0
        self.faults = []
        
        # Conversion constants
        self.COUNTS_PER_REV = 16384
        self.DEGREES_PER_COUNT = 360.0 / self.COUNTS_PER_REV
        
    def connect(self) -> bool:
        """
        Connect to the motor controller.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self.is_connected = True
            self.running = True
            
            # Start background listener
            self.listener_thread = threading.Thread(target=self._listen_loop)
            self.listener_thread.daemon = True
            self.listener_thread.start()
            
            # Start status monitor
            self.monitor_thread = threading.Thread(target=self._monitor_loop)
            self.monitor_thread.daemon = True
            self.monitor_thread.start()
            
            print(f"Connected to motor controller on {self.port} (CAN address: {self.device_id})")
            self._update_state(MotorState.ENABLED)
            return True
            
        except Exception as e:
            print(f"Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the motor controller."""
        self.running = False
        self.is_connected = False
        
        if self.serial_conn and self.serial_conn.is_open:
            # Disable motor before disconnecting
            self.disable()
            time.sleep(0.1)
            self.serial_conn.close()
        
        self._update_state(MotorState.DISABLED)
        print(f"Disconnected from motor controller (CAN address: {self.device_id})")
    
    def enable(self):
        """Enable the motor (wake it up from disabled state)."""
        # Motor enables automatically when commands are sent
        self._update_state(MotorState.ENABLED)
        print(f"Motor {self.device_id} enabled")
    
    def disable(self):
        """Disable the motor (free state, no torque)."""
        self._send_simple_command(0xCF)  # DISABLE_MOTOR
        self._update_state(MotorState.DISABLED)
        print(f"Motor {self.device_id} disabled")
    
    def emergency_stop(self):
        """Immediately stop the motor and disable output."""
        self.disable()
        print(f"EMERGENCY STOP: Motor {self.device_id} disabled")
    
    # === POSITION CONTROL ===
    
    def move_to(self, angle_degrees: float, wait: bool = False, timeout: float = 5.0):
        """
        Move to an absolute angle.
        
        Args:
            angle_degrees: Target angle in degrees
            wait: Whether to wait until movement completes
            timeout: Maximum time to wait in seconds
        """
        counts = self._degrees_to_counts(angle_degrees)
        self._send_position_command(0xC2, counts)  # ABS_POSITION_CONTROL
        self.target_angle = angle_degrees
        self.control_mode = ControlMode.POSITION
        self._update_state(MotorState.MOVING)
        
        print(f"Motor {self.device_id} moving to {angle_degrees}°")
        
        if wait:
            self.wait_for_move(timeout)
    
    def move_relative(self, angle_degrees: float, wait: bool = False, timeout: float = 5.0):
        """
        Move relative to current position.
        
        Args:
            angle_degrees: Relative angle in degrees
            wait: Whether to wait until movement completes
            timeout: Maximum time to wait in seconds
        """
        counts = self._degrees_to_counts(angle_degrees)
        self._send_position_command(0xC3, counts)  # REL_POSITION_CONTROL
        self.target_angle = self.current_angle + angle_degrees
        self.control_mode = ControlMode.POSITION
        self._update_state(MotorState.MOVING)
        
        print(f"Motor {self.device_id} moving {angle_degrees}° from current position")
        
        if wait:
            self.wait_for_move(timeout)
    
    def set_home(self):
        """Set current position as the home position."""
        self._send_simple_command(0xB1)  # SET_ORIGIN
        print(f"Motor {self.device_id} home position set")
    
    def go_home(self, wait: bool = False, timeout: float = 5.0):
        """
        Return to home position via shortest path.
        
        Args:
            wait: Whether to wait until movement completes
            timeout: Maximum time to wait in seconds
        """
        self._send_simple_command(0xC4)  # GO_HOME
        self.target_angle = 0.0
        self.control_mode = ControlMode.HOMING
        self._update_state(MotorState.HOMING)
        
        print(f"Motor {self.device_id} returning to home position")
        
        if wait:
            self.wait_for_move(timeout)
    
    def wait_for_move(self, timeout: float = 5.0, tolerance: float = 1.0):
        """
        Wait until the motor reaches its target position.
        
        Args:
            timeout: Maximum time to wait in seconds
            tolerance: Position tolerance in degrees
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            # Update position reading
            self.update_position()
            
            if abs(self.current_angle - self.target_angle) <= tolerance:
                self._update_state(MotorState.HOLDING)
                print(f"Motor {self.device_id} move completed. Current position: {self.current_angle:.1f}°")
                return
            
            time.sleep(0.1)
        
        print(f"Motor {self.device_id} move timeout. Current position: {self.current_angle:.1f}°, Target: {self.target_angle:.1f}°")
    
    # === SPEED CONTROL ===
    
    def set_speed(self, speed_rpm: float):
        """
        Set motor speed.
        
        Args:
            speed_rpm: Speed in RPM (positive or negative)
        """
        speed_raw = int(speed_rpm / 0.01)
        data_bytes = speed_raw.to_bytes(4, 'little', signed=True)
        self._send_command(0xC1, data_bytes, wait_response=False)  # SPEED_CONTROL
        
        self.control_mode = ControlMode.SPEED
        print(f"Motor {self.device_id} speed set to {speed_rpm} RPM")
    
    def stop(self):
        """Stop the motor (set speed to 0)."""
        self.set_speed(0)
        self._update_state(MotorState.HOLDING)
        print(f"Motor {self.device_id} stopped")
    
    # === TORQUE/CURRENT CONTROL ===
    
    def set_current(self, current_amps: float):
        """
        Set motor current (torque control).
        
        Args:
            current_amps: Current in Amps (positive or negative)
        """
        current_raw = int(current_amps / 0.001)
        data_bytes = current_raw.to_bytes(4, 'little', signed=True)
        self._send_command(0xC0, data_bytes, wait_response=False)  # CURRENT_CONTROL
        
        self.control_mode = ControlMode.CURRENT
        print(f"Motor {self.device_id} current set to {current_amps} A")
    
    # === SYSTEM COMMANDS ===
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get comprehensive motor status.
        
        Returns:
            Dictionary containing motor status information
        """
        payload = self._send_command(0xAE)  # READ_STATUS
        
        if payload and len(payload) >= 7:
            voltage_raw = int.from_bytes(payload[0:2], 'little')
            current_raw = int.from_bytes(payload[2:4], 'little')
            temperature = payload[4]
            operating_mode = payload[5]
            fault_code = payload[6]
            
            self.voltage = voltage_raw * 0.01
            bus_current = current_raw * 0.01
            self.temperature = temperature
            
            # Update faults
            self._update_faults(fault_code)
            
            return {
                'device_id': self.device_id,
                'voltage': self.voltage,
                'bus_current': bus_current,
                'temperature': self.temperature,
                'operating_mode': operating_mode,
                'faults': [fault.value for fault in self.faults],
                'state': self.state.value,
                'control_mode': self.control_mode.value,
                'current_angle': self.current_angle,
                'current_speed': self.current_speed,
                'target_angle': self.target_angle
            }
        return {}
    
    def update_position(self):
        """Update the current position reading."""
        angle_data = self._send_command(0xA3)  # READ_ANGLE
        
        if angle_data and len(angle_data) >= 6:
            single_turn_raw = int.from_bytes(angle_data[0:2], 'little')
            multi_turn_raw = int.from_bytes(angle_data[2:6], 'little', signed=True)
            
            single_turn_angle = single_turn_raw * self.DEGREES_PER_COUNT
            multi_turn_angle = multi_turn_raw * self.DEGREES_PER_COUNT
            
            self.current_angle = multi_turn_angle
            
            # Notify position update
            for callback in self.callbacks['position_update']:
                callback(self.current_angle, self.device_id)
    
    def get_versions(self) -> Dict[str, Any]:
        """
        Get firmware and hardware versions.
        
        Returns:
            Dictionary containing version information
        """
        payload = self._send_command(0xA0)  # READ_VERSION
        
        if payload and len(payload) >= 7:
            boot_version = int.from_bytes(payload[0:2], 'little')
            app_version = int.from_bytes(payload[2:4], 'little')
            hardware_version = int.from_bytes(payload[4:6], 'little')
            can_version = payload[6] if len(payload) > 6 else 0
            
            return {
                'device_id': self.device_id,
                'bootloader': boot_version,
                'firmware': app_version,
                'hardware': hardware_version,
                'can_protocol': can_version
            }
        return {}
    
    def clear_faults(self):
        """Clear all fault conditions."""
        self._send_command(0xAF)  # CLEAR_FAULT
        self.faults = [FaultType.NONE]
        self._update_state(MotorState.ENABLED)
        print(f"Motor {self.device_id} faults cleared")
    
    def restart(self):
        """Restart the motor controller."""
        data_bytes = b'\xFF\x00\xFF\x00\xFF\x00\xFF'
        self._send_command(0x00, data_bytes, wait_response=False)  # RESTART
        print(f"Motor {self.device_id} controller restarting...")
        self._update_state(MotorState.DISABLED)
    
    # === BRAKE CONTROL ===
    
    def brake_on(self):
        """Engage the brake."""
        result = self._send_command(0xCE, b'\x01')  # BRAKE_CONTROL on
        if result and result[0] == 0x01:
            print(f"Motor {self.device_id} brake engaged")
            return True
        return False
    
    def brake_off(self):
        """Release the brake."""
        result = self._send_command(0xCE, b'\x00')  # BRAKE_CONTROL off
        if result and result[0] == 0x00:
            print(f"Motor {self.device_id} brake released")
            return True
        return False
    
    def get_brake_status(self) -> bool:
        """Get brake status (True = engaged, False = released)."""
        result = self._send_command(0xCE, b'\xFF')  # BRAKE_CONTROL read
        if result and len(result) >= 1:
            return result[0] == 0x01
        return False
    
    # === CONFIGURATION METHODS ===
    
    def set_max_speed(self, max_rpm: float):
        """
        Set maximum speed for position mode.
        
        Args:
            max_rpm: Maximum speed in RPM
        """
        max_speed_raw = int(max_rpm / 0.01)
        data_bytes = max_speed_raw.to_bytes(4, 'little')
        self._send_command(0xB2, data_bytes)  # SET_MAX_SPEED
        print(f"Motor {self.device_id} maximum speed set to {max_rpm} RPM")
    
    def set_max_current(self, max_amps: float):
        """
        Set maximum current for position/speed mode.
        
        Args:
            max_amps: Maximum current in Amps
        """
        max_current_raw = int(max_amps / 0.001)
        data_bytes = max_current_raw.to_bytes(4, 'little')
        self._send_command(0xB3, data_bytes)  # SET_MAX_CURRENT
        print(f"Motor {self.device_id} maximum current set to {max_amps} A")
    
    # === CALLBACK MANAGEMENT ===
    
    def add_callback(self, event_type: str, callback: Callable):
        """
        Add a callback function for specific events.
        
        Args:
            event_type: One of 'state_change', 'fault', 'position_update', 'status_update'
            callback: Function to call when event occurs
        """
        if event_type in self.callbacks:
            self.callbacks[event_type].append(callback)
    
    def remove_callback(self, event_type: str, callback: Callable):
        """
        Remove a callback function.
        
        Args:
            event_type: Event type
            callback: Callback function to remove
        """
        if event_type in self.callbacks and callback in self.callbacks[event_type]:
            self.callbacks[event_type].remove(callback)
    
    # === PRIVATE METHODS ===
    
    def _send_command(self, command: int, data_bytes: bytes = b'', wait_response: bool = True) -> Optional[bytes]:
        """Send a command to the motor and optionally wait for response."""
        if not self.is_connected:
            print(f"Motor {self.device_id} not connected")
            return None
        
        data_length = 1 + len(data_bytes)
        if data_length > 15:
            raise ValueError(f"Data length {data_length} exceeds maximum of 15 bytes")
        
        length_byte = 0xC0 | data_length
        addr_bytes = self.device_id.to_bytes(2, 'little')
        
        packet = b'\xaa' + bytes([length_byte]) + addr_bytes + bytes([command]) + data_bytes + b'\x55'
        
        try:
            self.serial_conn.write(packet)
            
            if wait_response:
                time.sleep(0.1)
                if self.serial_conn.in_waiting > 0:
                    response = self.serial_conn.read(self.serial_conn.in_waiting)
                    return self._parse_response(response, command)
            
            return None
            
        except Exception as e:
            print(f"Motor {self.device_id} communication error: {e}")
            return None
    
    def _send_simple_command(self, command: int):
        """Send a command with no data bytes."""
        self._send_command(command, wait_response=False)
    
    def _send_position_command(self, command: int, counts: int):
        """Send a position control command."""
        data_bytes = counts.to_bytes(4, 'little', signed=True)
        self._send_command(command, data_bytes, wait_response=False)
    
    def _parse_response(self, data: bytes, expected_command: int) -> Optional[bytes]:
        """Parse response data."""
        if len(data) < 6:
            return None
        
        length_byte = data[1]
        data_length = length_byte & 0x0F
        
        if len(data) < 4 + data_length + 1:
            return None
        
        cmd = data[4]
        payload = data[5:5+data_length-1]
        
        if cmd != expected_command:
            print(f"Motor {self.device_id} unexpected command in response: expected {expected_command:02X}, got {cmd:02X}")
        
        return payload
    
    def _degrees_to_counts(self, angle_degrees: float) -> int:
        """Convert degrees to encoder counts."""
        return int(angle_degrees * (self.COUNTS_PER_REV / 360.0))
    
    def _counts_to_degrees(self, counts: int) -> float:
        """Convert encoder counts to degrees."""
        return counts * self.DEGREES_PER_COUNT
    
    def _update_state(self, new_state: MotorState):
        """Update motor state and notify callbacks."""
        if new_state != self.state:
            self.state = new_state
            for callback in self.callbacks['state_change']:
                callback(new_state, self.device_id)
    
    def _update_faults(self, fault_code: int):
        """Update fault status from fault code."""
        faults = []
        fault_types = {
            0: FaultType.VOLTAGE,
            1: FaultType.CURRENT,
            2: FaultType.TEMPERATURE,
            3: FaultType.ENCODER,
            6: FaultType.HARDWARE,
            7: FaultType.SOFTWARE
        }
        
        for bit, fault_type in fault_types.items():
            if fault_code & (1 << bit):
                faults.append(fault_type)
        
        if not faults:
            faults = [FaultType.NONE]
        
        if faults != self.faults:
            self.faults = faults
            if FaultType.NONE not in faults:
                self._update_state(MotorState.FAULT)
                for callback in self.callbacks['fault']:
                    callback(faults, self.device_id)
    
    def _listen_loop(self):
        """Background thread to listen for incoming messages."""
        while self.running and self.is_connected:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    # Handle spontaneous messages (like fault reports)
                    self._handle_incoming_data(data)
                time.sleep(0.01)
            except Exception as e:
                print(f"Motor {self.device_id} listener error: {e}")
                time.sleep(0.1)
    
    def _monitor_loop(self):
        """Background thread to monitor motor status."""
        while self.running and self.is_connected:
            try:
                status = self.get_status()
                if status:
                    for callback in self.callbacks['status_update']:
                        callback(status)
                time.sleep(0.5)  # Update every 500ms
            except Exception as e:
                print(f"Motor {self.device_id} monitor error: {e}")
                time.sleep(0.1)
    
    def _handle_incoming_data(self, data: bytes):
        """Handle incoming data from motor."""
        if len(data) >= 5:
            cmd = data[4]
            if cmd == 0xAE:  # Status report (may be fault)
                print(f"Motor {self.device_id} received status report")
                self.get_status()  # Update status


# === CONVENIENCE FUNCTIONS ===

def create_motor_controller(port: str, device_id: int = 1) -> CANMotorController:
    """
    Create and connect a motor controller.
    
    Args:
        port: Serial port name
        device_id: CAN device address (1-254)
    
    Returns:
        Connected CANMotorController instance
    
    Raises:
        ConnectionError: If connection fails
        ValueError: If device_id is invalid
    """
    motor = CANMotorController(port, device_id)
    if motor.connect():
        return motor
    else:
        raise ConnectionError(f"Failed to connect to motor on {port} (CAN address: {device_id})")

def create_multiple_motors(port: str, device_ids: List[int]) -> List[CANMotorController]:
    """
    Create and connect multiple motor controllers on the same port.
    
    Args:
        port: Serial port name
        device_ids: List of CAN device addresses
    
    Returns:
        List of connected CANMotorController instances
    """
    motors = []
    for device_id in device_ids:
        try:
            motor = create_motor_controller(port, device_id)
            motors.append(motor)
            print(f"Connected motor with CAN address {device_id}")
        except (ConnectionError, ValueError) as e:
            print(f"Failed to connect motor with CAN address {device_id}: {e}")
    
    return motors

def demo_single_motor(port: str, device_id: int = 1):
    """
    Run a demonstration with a single motor.
    
    Args:
        port: Serial port name
        device_id: CAN device address
    """
    print(f"=== CAN Motor Controller Demo (Motor {device_id}) ===")
    
    try:
        # Create and connect to motor
        motor = create_motor_controller(port, device_id)
        
        # Add some callbacks for demonstration
        def on_state_change(state, motor_id):
            print(f"Motor {motor_id} state changed to: {state.value}")
        
        def on_position_update(angle, motor_id):
            print(f"Motor {motor_id} position update: {angle:.1f}°")
        
        motor.add_callback('state_change', on_state_change)
        motor.add_callback('position_update', on_position_update)
        
        # Get system info
        versions = motor.get_versions()
        print(f"Motor {device_id} versions: {versions}")
        
        # Demo sequence
        print(f"\n1. Moving motor {device_id} to 0°")
        motor.move_to(0, wait=True)
        
        print(f"\n2. Moving motor {device_id} to 90°")
        motor.move_to(90, wait=True)
        
        print(f"\n3. Moving motor {device_id} to 180°")
        motor.move_to(180, wait=True)
        
        print(f"\n4. Relative move of -45° on motor {device_id}")
        motor.move_relative(-45, wait=True)
        
        print(f"\n5. Setting home position for motor {device_id}")
        motor.set_home()
        
        print(f"\n6. Returning motor {device_id} to home")
        motor.go_home(wait=True)
        
        # Show final status
        status = motor.get_status()
        print(f"\nMotor {device_id} final status:")
        for key, value in status.items():
            print(f"  {key}: {value}")
        
        # Cleanup
        motor.disconnect()
        print(f"\nMotor {device_id} demo completed successfully!")
        
    except Exception as e:
        print(f"Motor {device_id} demo failed: {e}")

def demo_multiple_motors(port: str, device_ids: List[int]):
    """
    Run a demonstration with multiple motors.
    
    Args:
        port: Serial port name
        device_ids: List of CAN device addresses
    """
    print(f"=== Multiple Motors Demo (Port: {port}, Devices: {device_ids}) ===")
    
    motors = create_multiple_motors(port, device_ids)
    
    if not motors:
        print("No motors connected successfully")
        return
    
    try:
        # Move all motors sequentially
        for i, motor in enumerate(motors):
            print(f"\n--- Controlling Motor {motor.device_id} ---")
            
            # Move to different positions for each motor
            target_angle = (i + 1) * 45  # 45°, 90°, 135°, etc.
            print(f"Moving motor {motor.device_id} to {target_angle}°")
            motor.move_to(target_angle, wait=True)
            
            # Get status
            status = motor.get_status()
            print(f"Motor {motor.device_id} position: {status.get('current_angle', 'Unknown')}°")
        
        # Return all motors to home position
        print(f"\n--- Returning all motors to home ---")
        for motor in motors:
            print(f"Returning motor {motor.device_id} to home")
            motor.go_home(wait=False)
        
        # Wait for all to complete
        time.sleep(3)
        
        # Disconnect all
        print(f"\n--- Disconnecting all motors ---")
        for motor in motors:
            motor.disconnect()
        
        print("Multiple motors demo completed successfully!")
        
    except Exception as e:
        print(f"Multiple motors demo failed: {e}")
        for motor in motors:
            motor.disconnect()


# === USAGE EXAMPLES ===

if __name__ == "__main__":
    # Example 1: Single motor with default address
    def example_single_default():
        """Example with single motor using default CAN address 1"""
        PORT = "/dev/tty.usbserial-2130"  # Change to your port
        demo_single_motor(PORT)  # Uses default address 1
    
    # Example 2: Single motor with specific address
    def example_single_custom():
        """Example with single motor using specific CAN address"""
        PORT = "COM3"  # Change to your port
        DEVICE_ID = 2  # Specific CAN address
        demo_single_motor(PORT, DEVICE_ID)
    
    # Example 3: Multiple motors
    def example_multiple():
        """Example with multiple motors on same port"""
        PORT = "/dev/ttyUSB0"  # Change to your port
        DEVICE_IDS = [1, 2, 3]  # Multiple CAN addresses
        demo_multiple_motors(PORT, DEVICE_IDS)
    
    # Example 4: Manual control
    def example_manual():
        """Manual control example"""
        PORT = "/dev/tty.usbserial-2130"
        DEVICE_ID = 1
        
        try:
            # Create motor instance
            motor = CANMotorController(PORT, DEVICE_ID)
            
            # Connect
            if motor.connect():
                print(f"Controlling motor {DEVICE_ID} on {PORT}")
                
                # Simple movements
                motor.move_to(45, wait=True)
                motor.move_relative(90, wait=True)
                motor.go_home(wait=True)
                
                # Cleanup
                motor.disconnect()
            else:
                print("Failed to connect")
                
        except Exception as e:
            print(f"Error: {e}")
    
    # Run one of the examples
    print("CAN Motor Controller Library - Usage Examples")
    print("Choose an example to run:")
    print("1. Single motor (default address 1)")
    print("2. Single motor (custom address)")
    print("3. Multiple motors")
    print("4. Manual control")
    
    try:
        choice = input("Enter choice (1-4): ").strip()
        PORT = input("Enter serial port (e.g., /dev/ttyUSB0, COM3): ").strip()
        
        if choice == "1":
            demo_single_motor(PORT)
        elif choice == "2":
            device_id = int(input("Enter CAN device ID (1-254): "))
            demo_single_motor(PORT, device_id)
        elif choice == "3":
            device_ids = input("Enter CAN device IDs (comma-separated, e.g., 1,2,3): ")
            device_list = [int(x.strip()) for x in device_ids.split(",")]
            demo_multiple_motors(PORT, device_list)
        elif choice == "4":
            example_manual()
        else:
            print("Invalid choice")
    except Exception as e:
        print(f"Error running example: {e}")

