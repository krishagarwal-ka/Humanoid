import serial

ser = serial.Serial('/dev/tty.wchusbserial58380003801', 115200)

while True:
    data = ser.readline()
    print(data.hex())

