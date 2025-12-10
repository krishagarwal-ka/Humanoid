import serial
import struct

ser = serial.Serial('/dev/cu.usbserial-140', 2000000)

def to_signed(val):
    return val - 65536 if val > 32767 else val

while True:
    if ser.read(1) == b'\x55':  # header
        type_byte = ser.read(1)
        data = ser.read(6)
        if len(data) < 6:
            continue
        vals = struct.unpack('<hhh', data)
        vals = [to_signed(v) for v in vals]

        if type_byte == b'\x51':  # acceleration
            ax, ay, az = [v / 32768 * 16 * 9.8 for v in vals]
        #    print(f"Accel: {ax:.2f}, {ay:.2f}, {az:.2f} m/s²")

        elif type_byte == b'\x52':  # angular velocity
            gx, gy, gz = [v / 32768 * 2000 for v in vals]
        #    print(f"Gyro: {gx:.2f}, {gy:.2f}, {gz:.2f} °/s")

        elif type_byte == b'\x53':  # angles
            pitch, roll, yaw = [v / 32768 * 180 for v in vals]
            print(f"Roll: {roll:.2f}   Pitch: {pitch:.2f}   Yaw: {yaw:.2f}")

