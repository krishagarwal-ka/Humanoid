import serial
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable, Tuple
from enum import Enum


class IMUDataType(Enum):
    ACCELERATION = 0x51
    ANGULAR_VELOCITY = 0x52
    ANGLES = 0x53


@dataclass
class IMUData:
    """Container for IMU sensor data"""
    acceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    angles: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    timestamp: float = 0.0


class IMUReader:
    """
    A user-friendly library for reading data from IMU sensors.
    
    Example usage:
    >>> imu = IMUReader(port='/dev/cu.usbserial-140')
    >>> imu.start_reading()
    >>> data = imu.get_latest_data()
    >>> print(f"Roll: {data.angles[0]:.2f}, Pitch: {data.angles[1]:.2f}, Yaw: {data.angles[2]:.2f}")
    >>> imu.stop_reading()
    """
    
    def __init__(self, port: str = '/dev/cu.usbserial-140', baudrate: int = 2000000):
        """
        Initialize the IMU reader.
        
        Args:
            port: Serial port name (e.g., '/dev/cu.usbserial-140' on macOS/Linux, 'COM3' on Windows)
            baudrate: Baud rate for serial communication (default: 2000000)
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_reading = False
        self.thread = None
        self.latest_data = IMUData()
        self.data_lock = threading.Lock()
        self.callbacks = {
            IMUDataType.ACCELERATION: [],
            IMUDataType.ANGULAR_VELOCITY: [],
            IMUDataType.ANGLES: []
        }
    
    def connect(self) -> bool:
        """
        Connect to the IMU sensor.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to IMU on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to IMU: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the IMU sensor."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected from IMU")
    
    def start_reading(self):
        """
        Start reading data from the IMU in a separate thread.
        """
        if not self.ser or not self.ser.is_open:
            if not self.connect():
                raise ConnectionError("Failed to connect to IMU")
        
        self.is_reading = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print("Started reading IMU data")
    
    def stop_reading(self):
        """
        Stop reading data from the IMU.
        """
        self.is_reading = False
        if self.thread:
            self.thread.join(timeout=2.0)
        self.disconnect()
        print("Stopped reading IMU data")
    
    def get_latest_data(self) -> IMUData:
        """
        Get the latest IMU data.
        
        Returns:
            IMUData: Latest sensor data
        """
        with self.data_lock:
            return self.latest_data
    
    def register_callback(self, data_type: IMUDataType, callback: Callable):
        """
        Register a callback function to be called when new data of the specified type is received.
        
        Args:
            data_type: Type of data to trigger callback (ACCELERATION, ANGULAR_VELOCITY, or ANGLES)
            callback: Function to call with the data as argument
        """
        self.callbacks[data_type].append(callback)
    
    def _to_signed(self, val: int) -> int:
        """Convert unsigned 16-bit value to signed."""
        return val - 65536 if val > 32767 else val
    
    def _process_data(self, data_type: IMUDataType, raw_data: bytes):
        """Process raw sensor data and update latest data."""
        if len(raw_data) < 6:
            return
        
        vals = struct.unpack('<hhh', raw_data)
        vals = [self._to_signed(v) for v in vals]
        
        with self.data_lock:
            self.latest_data.timestamp = time.time()
            
            if data_type == IMUDataType.ACCELERATION:
                # Convert to m/s²
                self.latest_data.acceleration = tuple(v / 32768 * 16 * 9.8 for v in vals)
                self._notify_callbacks(data_type, self.latest_data.acceleration)
                
            elif data_type == IMUDataType.ANGULAR_VELOCITY:
                # Convert to °/s
                self.latest_data.angular_velocity = tuple(v / 32768 * 2000 for v in vals)
                self._notify_callbacks(data_type, self.latest_data.angular_velocity)
                
            elif data_type == IMUDataType.ANGLES:
                # Convert to degrees
                self.latest_data.angles = tuple(v / 32768 * 180 for v in vals)
                self._notify_callbacks(data_type, self.latest_data.angles)
    
    def _notify_callbacks(self, data_type: IMUDataType, data):
        """Notify registered callbacks of new data."""
        for callback in self.callbacks[data_type]:
            try:
                callback(data)
            except Exception as e:
                print(f"Error in callback: {e}")
    
    def _read_loop(self):
        """Main reading loop running in separate thread."""
        while self.is_reading and self.ser and self.ser.is_open:
            try:
                if self.ser.read(1) == b'\x55':  # Header byte
                    type_byte = self.ser.read(1)
                    data = self.ser.read(6)
                    
                    if type_byte == b'\x51':
                        self._process_data(IMUDataType.ACCELERATION, data)
                    elif type_byte == b'\x52':
                        self._process_data(IMUDataType.ANGULAR_VELOCITY, data)
                    elif type_byte == b'\x53':
                        self._process_data(IMUDataType.ANGLES, data)
                        
            except serial.SerialException as e:
                print(f"Serial communication error: {e}")
                break
            except Exception as e:
                print(f"Unexpected error: {e}")
                continue


# Convenience functions for quick usage
def create_imu_reader(port: str = '/dev/cu.usbserial-140', baudrate: int = 2000000) -> IMUReader:
    """Create and return a configured IMUReader instance."""
    return IMUReader(port, baudrate)


def print_angles_callback(angles):
    """Example callback function for printing angles."""
    roll, pitch, yaw = angles
    print(f"Roll: {roll:7.2f}°  Pitch: {pitch:7.2f}°  Yaw: {yaw:7.2f}°")


def print_acceleration_callback(acceleration):
    """Example callback function for printing acceleration."""
    ax, ay, az = acceleration
    print(f"Accel: X={ax:7.2f}, Y={ay:7.2f}, Z={az:7.2f} m/s²")


def print_gyro_callback(angular_velocity):
    """Example callback function for printing angular velocity."""
    gx, gy, gz = angular_velocity
    print(f"Gyro:  X={gx:7.2f}, Y={gy:7.2f}, Z={gz:7.2f} °/s")


# Example usage
if __name__ == "__main__":
    # Create IMU reader
    imu = create_imu_reader()
    
    # Register callbacks for different data types
    imu.register_callback(IMUDataType.ANGLES, print_angles_callback)
    imu.register_callback(IMUDataType.ACCELERATION, print_acceleration_callback)
    imu.register_callback(IMUDataType.ANGULAR_VELOCITY, print_gyro_callback)
    
    try:
        # Start reading
        imu.start_reading()
        
        # Keep the main thread alive
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nStopping IMU reader...")
    finally:
        imu.stop_reading()

