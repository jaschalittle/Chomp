from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.animation as animation
import pandas as pd
import math
import collections
import itertools
import time
import multiprocessing
import serial
import sys


datastream = [-10, -10, 10, 10]


# for data printed to debug
def stream_data(serial_port, q):
    ser = serial.Serial(serial_port, 115200)
    # fig = plt.figure()
    # ax = plt.gca()
    # scatter = ax.scatter([1, 1, 1], [1, 1, 1], c=["blue", "green", "red"], s=[200, 200, 200], edgecolor="none")
    # ax.set_xlim([-10, 10])
    # ax.set_ylim([-10, 10])
    while True:
        try:
            line = ser.readline().rstrip()
            new_data = line.split("\t")
            datastream = map(int, new_data[:4])
            # error = int(error) / 255 * 16 - 8
            # scatter.set_offsets(new_data[0:5:2], new_data[1:6:2])
            # line.set_xdata(new_data[0:5:2])
            # line.set_ydata(new_data[1:6:2])
            
            q.put(datastream)
            lead = ((datastream[2] - datastream[0]) ** 2 + (datastream[3] - datastream[1]) ** 2) ** 0.5
            new_data.append(lead)
            print line
            # print "{:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7.3f} cm".format(*new_data)
        except ValueError as err:
            print "/".join(map(str, new_data))
            print err
        except IndexError as err:
            print "/".join(map(str, new_data))
            print err


class SubplotAnimation(animation.TimedAnimation):
    def __init__(self, serial_port):
        self.line = None

        # make list of shared array objects here so that another thread can read serial data.
        # if this thread continues plotting data in array, it should keep itself up to date as long as
        # its refresh rate exceeds data streaming rate

        # self.datastreams = [multiprocessing.Array('i', num_points) for _ in 48]
        self.q = multiprocessing.Queue()
        self.data_stream = multiprocessing.Process(target=stream_data, args=(serial_port, self.q))
        fig = plt.figure()
        ax = plt.gca()
        # line, = ax.plot(datastream, range(num_points), marker='.', markersize=3, linewidth=0)
        # leddar_x, leddar_y, pred_x, pred_y, lead_x, lead_y = datastream
        leddar_x, leddar_y, lead_x, lead_y = datastream
        # scatter = ax.scatter([1, 1, 1], [1, 1, 1], c=["blue", "green", "red"], s=[200, 200, 200], edgecolor="none")
        line1, = ax.plot([leddar_x], [leddar_y], markersize=20, linewidth=0, marker="o", color="green", alpha=0.6)
        # ax.plot([0, 500], [0, 500], color="black")
        # ax.plot([0,-500], [0, -500], color="black")
        # line2, = ax.plot([pred_x], [pred_y], markersize=20, marker="o", linewidth=0, color="blue", alpha=0.6)
        line3, = ax.plot([lead_x], [lead_y], markersize=20, marker="o", linewidth=0, color="red", alpha=0.6)
        plt.title("Leddar object tracking", fontweight="bold", fontsize=24)
        self.lines = [line1, line3]
        # self.line = line1
        # self.scatter = scatter
        ax.set_xlim([-700, 700])
        ax.set_ylim([-300, 900])
        # ax.xaxis.set_ticks(range(-8, 9))


        # for ax in axes_array:
        #     plt.setp(ax.get_xticklabels(), rotation=90, fontsize=16)

        # for ax in axes_array[1:]:
        #     plt.setp(ax.get_yticklabels(), visible=False)

        ax.set_ylabel("y coordinate (cm)", fontsize=20, fontweight="bold", labelpad=50)
        ax.set_xlabel("x coordinate (cm)", fontsize=20, fontweight="bold", labelpad=50)
        # ax.yaxis.set_ticks([0, num_points/2, num_points])
        # axes_array[0].set_xticklabels([-num_points/sensor_freq/3, -num_points/sensor_freq/2/3, 0])
        # axes_array[0].set_yticklabels([-num_points/sensor_freq, -num_points/sensor_freq/2, 0])
        # plt.setp(axes_array[0].get_yticklabels(), fontsize=16)

        animation.TimedAnimation.__init__(self, fig, interval=20, blit=False)

        # start thread for each axis, need to pass it a shared array, segment number

    def _draw_frame(self, framedata):
        # for i, line in enumerate(self.lines):
        #     for _ in range(50 // update_freq):
        #         datastreams[i].append(data[i].next())
        #     line.set_xdata(datastreams[i])

        while not self.q.empty():
            data = self.q.get()
            leddar_x, leddar_y, lead_x, lead_y = data
            try:
                # self.line.set_xdata([leddar_x])
                # self.line.set_ydata([leddar_y])
                self.lines[0].set_xdata([leddar_x])
                self.lines[0].set_ydata([leddar_y])
                self.lines[1].set_xdata([lead_x])
                self.lines[1].set_ydata([lead_y])
                # self.lines[2].set_xdata([lead_x])
                # self.lines[2].set_ydata([lead_y])
                # self.line.set_xdata([20,])
                # self.line.set_ydata([20,])
                # self.scatter.set_offsets(data)
            except TypeError as err:
                print data
                print err
                self.lines[0].set_xdata(leddar_x)
                self.lines[0].set_ydata(leddar_y)
                self.lines[1].set_xdata(lead_x)
                self.lines[1].set_ydata(lead_y)
                # self.lines[2].set_xdata(lead_x)
                # self.lines[2].set_ydata(lead_y)

        self._drawn_artists = self.line
        # print time.time()

    def new_frame_seq(self):
        return iter(range(1000))

    def _init_draw(self):
        # self.lines.set_xdata(datastream)
        pass


# stream_data(sys.argv[1], multiprocessing.Queue())
# ani = SubplotAnimation("/dev/tty.usbmodem1411")
ani = SubplotAnimation(sys.argv[1])
# ani = SubplotAnimation("/dev/tty.SLAB_USBtoUART")
# ani = SubplotAnimation("/dev/tty.usbserial-DA01MEGZ")
# ani = SubplotAnimation("/dev/tty.usbserial-DA01R4QX")
ani.data_stream.start()
#ani.save('test_sub.mp4')
plt.show()