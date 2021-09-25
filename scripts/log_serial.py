from datetime import datetime
import serial
import sys

print(sys.argv)

with open(sys.argv[3], "a") as f:
    with serial.Serial(port=sys.argv[1], baudrate=sys.argv[2]) as ser:
        while ser.isOpen():
            now = datetime.now()
            line = now.strftime("%d/%m/%Y %H:%M:%S") + " " + ser.readline().decode("ascii")
            print(line[0:-1])
            f.write(line)