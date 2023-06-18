#!/usr/bin/env python3

import serial
import csv
import time
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/serial0',115200)
t_list = []
f_list = []
p_list = []
f = 0.0
t = 0.0
c = 'white'

plt.ion()

fig, ax = plt.subplots(2, 1, figsize=(10, 7))
ax1 = ax[1]
ax2 = ax[0]
li0, = ax1.plot(t_list, f_list, color='orange')
li1, = ax2.plot(t_list, p_list, color='red')

ax1.set_title('Flow vs Time')
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Flow Frequency [Hz]')
ax1.set_facecolor('#262626')
ax1.title.set_color(c)
ax1.xaxis.label.set_color(c)        
ax1.yaxis.label.set_color(c)          
ax1.tick_params(axis='x', colors=c)    
ax1.tick_params(axis='y', colors=c)  
ax1.spines['left'].set_color(c)        
ax1.spines['top'].set_color(c)
ax1.spines['right'].set_color(c)        
ax1.spines['bottom'].set_color(c)

ax2.set_title('Pressure vs Time')
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Pressure [V]')
ax2.set_facecolor('#262626')
ax2.title.set_color(c)
ax2.xaxis.label.set_color(c)        
ax2.yaxis.label.set_color(c)          
ax2.tick_params(axis='x', colors=c)    
ax2.tick_params(axis='y', colors=c)  
ax2.spines['left'].set_color(c)        
ax2.spines['top'].set_color(c)
ax2.spines['right'].set_color(c)        
ax2.spines['bottom'].set_color(c)
fig.set_facecolor('black')
fig.tight_layout()

with open('sensor_data.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    t_end = 0
    f_str = 0
    p_str = 0
    writer.writerow(["Time", "Flow Frequency", "Pressure"])

    while True:
    	
    	if(len(t_list) > 0):
    	    ax1.set_xlim([min(t_list)*0.99, max(t_list)*1.01])
    	    ax2.set_xlim([min(t_list)*0.99, max(t_list)*1.01])
    	if(len(p_list) > 0):
            ax1.set_ylim([min(p_list)*0.95, max(p_list)*1.05])
    	if(len(f_list) > 0):
            ax2.set_ylim([min(f_list)*0.95, max(f_list)*1.05])
    	line = ser.readline().decode('utf-8')

    	for i in range(len(line)):
            if line[i] == ')':
                t_end = i
            if line[i] == 'F':
                f_str = i + 1
            if line[i] == 'P':
                p_str = i + 1

    	t = line[0:t_end]
    	f = line[f_str: f_str+1]
    	p = line[p_str: p_str+2]
    	

    	#f = float(line[f_str:p_str-2])+0.0
    	#p = float(line[p_str:p_str+2])+0.0
    	writer.writerow([t,f,p])
    	t_list.append(float(t)*-1)
    	f_list.append(float(f))
    	p_list.append(float(p))
    	li0.set_xdata(t_list)
    	li0.set_ydata(p_list)
    	li1.set_xdata(t_list)
    	li1.set_ydata(f_list)
    	plt.draw()
    	#fig.canvas.draw()
    	#fig.canvas.flush_events()
    	#print(line)
