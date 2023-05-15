#!/usr/bin/env python3

import serial
import csv

ser = serial.Serial('/dev/serial0',9600)

with open('sensor_data.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    t_end = 0
    f_str = 0
    p_str = 0
    writer.writerow(["Time", "Flow Frequency", "Pressure"])

    while True:
        line = ser.readline().decode('utf-8')
        for i in range(len(line)):
            if line[i] == ')':
                t_end = i
            if line[i] == 'F':
                f_str = i + 3
            if line[i] == 'P':
                p_str = i + 3
            t = line[0:t_end]
            f = line[f_str:p_str-7]
            p = line[p_str:len(line)-1]
        writer.writerow([t,f,p])
        print(line)
