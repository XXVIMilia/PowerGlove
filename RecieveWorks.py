#!/usr/bin/env python3
import serial
import bluetooth
import time

bd_addr ="DC:A6:32:29:AF:F3"

port=2
sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((bd_addr,port))

i= 0
x_accelration=[]
y_accelration=[]
z_accelration=[]
x_orientation=[]
y_orientation=[]
z_orientation=[]

if __name__ == '__main__':
    try:
        
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=5)
    except:
        ser = serial.Serial('/dev/ttyACM1', 115200, timeout=5)

    ser.flush()
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode().rstrip() # reads in serial data from arduino
##            print(line)
            dataIn = line.split("!", 5)
##            print(dataIn)
            while(line.count('!') < 5):
                line = ser.readline().decode().rstrip()
                dataIn = line.split("!", 5)

            #Goes through incoming data and organizes it
            x_ori = dataIn[0]
            y_ori = dataIn[1]
            z_ori = dataIn[2]
            x_acc = dataIn[3]
            y_acc = dataIn[4]
            z_acc = dataIn[5]
            # stores every data point into its own infinetly long list
            x_accelration.append(x_acc)
            y_accelration.append(y_acc)
            z_accelration.append(z_acc)
            x_orientation.append(x_ori)
            y_orientation.append(y_ori)
            z_orientation.append(z_ori)
            outputt = 0
            if(x_ori != "0"):
                for i in range(0, 90, 1):
                    ouputt=str(i)
                    time.sleep(.01)
                    sock.send(ouputt)
                    print(ouputt)
##            print("x acceleration: ")
##            print(x_acc)
##            print("y acceleration: ")
##            print(y_acc)
##            print(*x_accelration)
            #dataOut = x_acc+ " "+ y_acc + " " + x_ori
            #print("this is what Hester gets: "+ dataOut)
            
sock.close()
ser.close()
