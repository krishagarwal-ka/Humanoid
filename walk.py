from motor_control import *
from imu import *
from time import *

motor = CANMotorController("/dev/tty.usbserial-210", 1)
motor.connect()
motor.go_home()
sleep(2)
motor.move_to(30)
sleep(2)
motor.go_home()
print(motor.current_angle)
print(motor.voltage)

