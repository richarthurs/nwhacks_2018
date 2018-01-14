import serial
ser = serial.Serial('/dev/serial0', 115200, timeout = 1)

print ser.name


for i in range(0,5,1):
	ser.write(str(i))

ser.close()	
