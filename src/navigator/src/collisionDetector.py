
import serial

port_1 = '/dev/serial/by-path/platform-3610000.xhci-usb-0:2.2.3:1.0'
port_2 = '/dev/ttyACM0'

with serial.Serial(port=port_1, baudrate=9600, timeout=10) as ser:
	while True:
		print(ser.readline())