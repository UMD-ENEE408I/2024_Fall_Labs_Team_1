import serial

ser = serial.Serial('COM18', 115200)

ser.write(b'Hello, Arduino!')

ser.close()