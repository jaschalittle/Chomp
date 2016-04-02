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

sensor_freq = 50.0

num_points = 1000

# in Hz. should be even divisor of 50 and 1000
update_freq = 5

# data = [list() for _ in range(16)]

# with open("leddar_data.txt") as fh:
#     for line in fh:
#         line = line.rstrip().split('\t')
#         for segment, detection in enumerate(line):
#             seg_num, distance = detection.split('/')
#             if distance == "nan":
#                 data[segment].append(np.nan)
#             else:
#                 data[segment].append(int(distance))

# datastreams = [collections.deque(data[segment][:num_points], maxlen=num_points) for segment in range(16)]
datastream = [0, 0, 1, 1, 2, 2]


# for byte data from Xbee
# def stream_data(serial_port, q):
#     ser = serial.Serial(serial_port, 115200)
#     raw_bytes = list()
#     data_to_send = [np.nan for _ in range(16)]
#     while True:
#         byte = ord(ser.read())
#         print byte >> 4,
#         if byte == 0 and raw_bytes[-1] == 0:
#             print "STOP"
#             # print new_data[:-1]
#             for i in xrange(32, 2):
#                 segment = raw_bytes[i] >> 4
#                 distance = (raw_bytes[i] & 0xFF) + raw_bytes[i + 1]
#                 if distance < 0xFFF:
#                     data_to_send[segment] = distance
#                 # print segment, distance,
#             # print "PARSED"
#             angle = raw_bytes[16] / 255 * 16 - 8
#             # print angle
#             q.put(data_to_send)
#             raw_bytes = list()
#             data_to_send = [np.nan for _ in range(16)]
#         else:
#             raw_bytes.append(byte)

# for data printed to debug
def stream_data(serial_port, q):
    ser = serial.Serial(serial_port, 57600)
    while True:
        error = float(ser.readline().rstrip())
        error = int(error) / 255 * 16 - 8
        q.put(error)
        print error


# This example uses subclassing, but there is no reason that the proper
# function couldn't be set up and then use FuncAnimation. The code is long, but
# not really complex. The length is due solely to the fact that there are a
# total of 9 lines that need to be changed for the animation as well as 3
# subplots that need initial set up.
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
        leddar_x, leddar_y, pred_x, pred_y, lead_x, lead_y = datastream
        line1, = ax.plot([leddar_x], [leddar_y], markersize=5, linewidth=0)
        line2, = ax.plot([pred_x], [pred_y], markersize=5, linewidth=0)
        line3, = ax.plot([lead_x], [lead_y], markersize=5, linewidth=0)
        plt.title("Leddar returns", fontweight="bold", fontsize=24)
        self.lines = [line1, line2, line3]
        ax = plt.gca()
        ax.set_xlim([-10, 10])
        ax.set_ylim([-10, 10])
        # ax.xaxis.set_ticks(range(-8, 9))


        # for ax in axes_array:
        #     plt.setp(ax.get_xticklabels(), rotation=90, fontsize=16)

        # for ax in axes_array[1:]:
        #     plt.setp(ax.get_yticklabels(), visible=False)

        ax.set_ylabel("Time (seconds ago)", fontsize=20, fontweight="bold", labelpad=50)
        ax.set_xlabel("Error", fontsize=20, fontweight="bold", labelpad=50)
        # ax.yaxis.set_ticks([0, num_points/2, num_points])
        # axes_array[0].set_xticklabels([-num_points/sensor_freq/3, -num_points/sensor_freq/2/3, 0])
        # axes_array[0].set_yticklabels([-num_points/sensor_freq, -num_points/sensor_freq/2, 0])
        # plt.setp(axes_array[0].get_yticklabels(), fontsize=16)

        animation.TimedAnimation.__init__(self, fig, interval=1000 // update_freq, blit=False)

        # start thread for each axis, need to pass it a shared array, segment number

    def _draw_frame(self, framedata):
        # for i, line in enumerate(self.lines):
        #     for _ in range(50 // update_freq):
        #         datastreams[i].append(data[i].next())
        #     line.set_xdata(datastreams[i])

        while not self.q.empty():
            data = self.q.get()
            datastream.append(data)
        
        leddar_x, leddar_y, pred_x, pred_y, lead_x, lead_y = datastream
        self.lines[0].set_xdata(leddar_x)
        self.lines[0].set_ydata(leddar_y)
        self.lines[1].set_xdata(pred_x)
        self.lines[1].set_ydata(pred_y)
        self.lines[2].set_xdata(lead_x)
        self.lines[2].set_ydata(lead_y)

        self._drawn_artists = self.lines
        # print time.time()

    def new_frame_seq(self):
        return iter(range(num_points))

    def _init_draw(self):
        # self.lines.set_xdata(datastream)
        pass


# ani = SubplotAnimation("/dev/tty.usbmodem1411")
ani = SubplotAnimation(sys.argv[1])
# ani = SubplotAnimation("/dev/tty.SLAB_USBtoUART")
# ani = SubplotAnimation("/dev/tty.usbserial-DA01MEGZ")
# ani = SubplotAnimation("/dev/tty.usbserial-DA01R4QX")
ani.data_stream.start()
#ani.save('test_sub.mp4')
plt.show()