"""
ldr.py
Display analog data from Arduino using Python (matplotlib)
Author: Mahesh Venkitachalam
Website: electronut.in
"""

import sys, serial, argparse
import numpy as np
from time import sleep
from collections import deque

import matplotlib.pyplot as plt 
import matplotlib.animation as animation

    
# plot class
class AnalogPlot:
  # constr
    def __init__(self, strPort, maxLen):
        # open serial port
        self.ser = serial.Serial(strPort, 115200)

        self.ax = deque([0]*maxLen)
        self.ay = deque([0]*maxLen)
        self.maxLen = maxLen
        self.time = 0

    # add to buffer
    def addToBuf(self, buf, val):
        if len(buf) < self.maxLen:
          buf.append(val)
        else:
          buf.pop()
          buf.appendleft(val)

    # add data
    def add(self, data):
        #       assert(len(data) == 2)
        for datum in data:
            self.time += 1
            self.addToBuf(self.ax, self.time)
            self.addToBuf(self.ay, datum)

    # update plot
    def update(self, frameNum, a0, a1):
        try:
            line = self.ser.readline()
            data = [int(val) for val in line.split()]
#             print data
            #           if(len(data) == 2):
            self.add(data)
            a0.set_data(range(self.maxLen), self.ax)
            a1.set_data(range(self.maxLen), self.ay)
        except KeyboardInterrupt:
            print('exiting')

        return a0, 

    # clean up
    def close(self):
        # close serial
        self.ser.flush()
        self.ser.close()  


def main():
    plot = AnalogPlot('/dev/tty.usbmodem1451', 1000)
    fig = plt.figure()
    ax = plt.axes(xlim=(0, 1000), ylim=(0, 2000))
    a0, = ax.plot([], [])
    a1, = ax.plot([], [])
    anim = animation.FuncAnimation(fig, plot.update, 
                                 fargs=(a0, a1), 
                                 interval=50)

    # show plot
    plt.show()

if __name__ == '__main__':
  main()