import bluetooth
import serial
import time

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
port = 2
sock.bind(("",port))
sock.listen(1)

client_S, addr = sock.accept()

while 1:
    data = client_S.recv(1024)
    toian=data.decode()
    print(toian)

    if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.flush()
    while True:
        ser.write(toian)
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        time.sleep(1)

        
client_S.close()
sock.close()

#!/usr/bin/env python3

