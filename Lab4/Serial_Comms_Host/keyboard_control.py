import serial
import keyboard

ser = serial.Serial('COM18', 115200)

mapping = {
    'up': 'F\n',
    'down': 'B\n',
    'left': 'L\n',
    'right': 'R\n',
    'space': 'S\n'
}

try:
    while True:
        for key in mapping:
            if keyboard.is_pressed(key):
                command = mapping[key]
                ser.write(command.encode())
                print(f'Sent: {command.strip()}')

except KeyboardInterrupt:
    ser.close()
    print("Closed")
