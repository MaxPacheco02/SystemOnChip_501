#!/usr/bin/env python3

from PyQt5.QtWidgets import QWidget, QApplication, QGridLayout
import pyqtgraph as pg
import sys
import serial
import csv

ser = serial.Serial('/dev/serial0',115200)
t_list = []
f_list = []
p_list = []
f = 0.0
t = 0.0

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
    	my_app.graph(t,f)
    	print(line)


class GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('RobertShaw Reto - Lectura de Sensores')
        self.setStyleSheet('background-color: #262626;')
        self.resize(650, 650)

        self.box = QGridLayout()
        self.box.setSpacing(5)
        self.box.setContentMargins(5,5,5,5)
        self.setLayout(self.box)

    def graph(self, x, y):
        plt = pg.PlotWidget('Grafica Lineal')
        plt.plot(x, y, Symbol = 'star', symblBrush = "#00AAAA", color = "00AAAA",
        pen = "#00AAAA", width = 2, SymbolPen = "#00AAAA")
        sef.box.addWidget(plt,0,1)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    my_app = GUI()
    my_app.show()
    sys.exit(app.exec_())
