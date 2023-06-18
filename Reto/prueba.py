#!/usr/bin/env python3

import serial
import csv
import time
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/serial0',9600)
time = 0.0
t_list = []
f_list = []
p_list = []
v_list = []
k_list = []

c = 'white'
plt.ion()

# url of the images
offUrl = "images/SystemOnChip/switch-off.png"
onUrl = "images/SystemOnChip/switch-on.png"

# import the image
onPic = plt.imread(onUrl, "r")
offPic = plt.imread(offUrl, "r")

# Create your subplots
fig, ax = plt.subplots(2, 4, figsize=(12, 7))
#plt.figure(figsize=(12,6))
ax1 = plt.subplot(2,2,1)
ax2 = plt.subplot(2,2,2)
ax3 = plt.subplot(2,7,8)
ax4 = plt.subplot(2,7,9)
ax5 = plt.subplot(2,7,10)
ax6 = plt.subplot(2,7,12)
ax7 = plt.subplot(2,7,14)
axes = [ax1, ax2, ax3, ax4, ax5, ax6, ax7]


li0, = ax1.plot(t_list, f_list, color='orange')
li1, = ax2.plot(t_list, p_list, color='red')

#Freq table
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

#Pressure table
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


#IN switch
ax3.set_title('IN', fontsize = 20)
ax3.title.set_color(c)
ax3.imshow(onPic)
ax3.axis('off')

#OUT switch
ax4.set_title('OUT', fontsize = 20)
ax4.title.set_color(c)
ax4.imshow(offPic)
ax4.axis('off')

#PUMP switch
ax5.set_title('PUMP', fontsize = 20)
ax5.title.set_color(c)
ax5.imshow(onPic)
ax5.axis('off')

#Kill switch
ax6.set_title('Kill Button', fontsize = 20)
ax6.title.set_color(c)
ax6.imshow(offPic)
ax6.axis('off')

#Time
ax7.set_title('Time', fontsize = 20)
ax7.title.set_color(c)
ax7.axis('off')

fig.set_facecolor('black')
#fig.tight_layout()

with open('sensor_data.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    t_end = 0
    f_str = 0
    v_str = 0
    k_str = 0
    p_str = 0
    writer.writerow(["Time", "Flow Frequency", "Pressure", "Valves", "Kill Button"])

    while True:
        if(len(t_list) > 0):
            ax1.set_xlim([0, max(t_list)*1.01])
            ax2.set_xlim([0, max(t_list)*1.01])
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
            if line[i] == 'V':
                v_str = i + 1
            if line[i] == 'K':
                k_str = i + 1
 
            t = line[0:t_end]
            f = line[f_str:p_str-2]
            p = line[p_str:k_str-2]
            v = line[v_str:f_str-2]
            k = line[k_str:len(line)]
            print(t)
            print(f)
            print(v)
            print(p)
            print(k)
            
        writer.writerow([t,f,p,v,k])
        t_list.append(float(t))
        f_list.append(float(f))
        p_list.append(float(p))
        v_list.append(float(v))
        k_list.append(float(k))
        li0.set_xdata(time)
        li0.set_ydata(p_list)
        li1.set_xdata(time)
        li1.set_ydata(f_list)
        time = time + 0.1
        fig.canvas.draw()
        fig.canvas.flush_events()
        print(line)
        
        
def option_one():
    ax3.imshow(offPic)
    ax4.imshow(offPic)
    ax5.imshow(offPic)

def option_two():
    ax3.imshow(onPic)
    ax4.imshow(offPic)
    ax5.imshow(onPic)

def option_three():
    ax3.imshow(offPic)
    ax4.imshow(onPic)
    ax5.imshow(offPic)

def switch_case(v):
    switcher = {
        1: option_one,
        2: option_two,
        3: option_three
    }
