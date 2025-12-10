import serial

serial_port="/dev/tty.usbserial-2130"

class Actuator:
    def __init__(self, address):
        self.ser = serial.Serial(serial_port, 2000000)
        self.address = address
        
    def write(self, degrees):
        counts = int(degrees*(16384/360))
        packet = bytes([
            0xaa, 0xc5, self.address, 0x00, 0xc2,
            counts & 0xFF,
            (counts >> 8) & 0xFF,
            (counts >> 16) & 0xFF, 
            (counts >> 24) & 0xFF,
            0x55])
        self.ser.write(packet)

a = Actuator(0x01)

while True:
    a.write(0)

