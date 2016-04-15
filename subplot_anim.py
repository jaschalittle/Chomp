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
datastreams = [collections.deque([0] * num_points, maxlen=num_points) for segment in range(16)]

# data = [itertools.cycle(segment) for segment in data]

# def update(i, line):
#     for _ in range(50 // update_freq):
#         datastreams[i].append(data[i].next())
#     lines[i].set_xdata(datastreams[i])

# def stream_data(serial_port, q):
#     ser = serial.Serial(serial_port, 115200)
#     while True:
#         new_data = [list() for _ in range(16)]
#         line = ser.readline()
#         detections = line.rstrip().split()
#         for detection in detections[1:]:
#             print line.rstrip()
#             if detection[:4] in ["LEFT", "RIGH", "Near", "erro"]:
#                 pass
#             else:
#                 try:
#                     segment, distance = detection.split("/")
#                     new_data[int(segment)].append(int(distance))
#                 except ValueError as err:
#                     print err
#                     print line
#         data_to_send = [segment + [np.nan] * (3 - len(segment)) for segment in new_data]
#         # print data_to_send
#         q.put(data_to_send[::-1])

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

    # data = [itertools.cycle(segment) for segment in data]
    # while True:
    #     to_q = list()
    #     for segment in data:
    #         to_q.append((segment.next(), np.nan, np.nan))
    #     q.put(to_q)
    #     time.sleep(1/sensor_freq)

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
    ser = serial.Serial(serial_port, 115200)
    while True:
        line = ser.readline()
        line = [field for field in line.rstrip().split() if field not in ["LEFT", "RIGHT"]]
        if len(line) == 17:
            detections = [int(field.split("/")[1]) for field in line[:16]]
            q.put(detections[:16])
            print " ".join(map(str, detections)),
            print int(line[-1]) / 255 * 16 - 8


# This example uses subclassing, but there is no reason that the proper
# function couldn't be set up and then use FuncAnimation. The code is long, but
# not really complex. The length is due solely to the fact that there are a
# total of 9 lines that need to be changed for the animation as well as 3
# subplots that need initial set up.
class SubplotAnimation(animation.TimedAnimation):
    def __init__(self, serial_port):
        self.lines = list()

        # make list of shared array objects here so that another thread can read serial data.
        # if this thread continues plotting data in array, it should keep itself up to date as long as
        # its refresh rate exceeds data streaming rate

        # self.datastreams = [multiprocessing.Array('i', num_points) for _ in 48]
        self.q = multiprocessing.Queue()
        self.data_stream = multiprocessing.Process(target=stream_data, args=(serial_port, self.q))

        fig, axes_array = plt.subplots(1, 16, figsize=(25, 20), subplot_kw={"axisbg": "white"})
        fig.suptitle("Leddar returns", fontweight="bold", fontsize=24)

        axes_array = axes_array.flatten()
        for plot_num, segment in enumerate(datastreams):
            ax = axes_array[plot_num]
            line, = ax.plot([0] * num_points, range(num_points), marker='.', markersize=3, linewidth=0)
            self.lines.append(line)
            # dataseries = pd.Series(datastreams[plot_num])
            # mean = dataseries.mean()
            # stdev = dataseries.std()
            # low_lim = int(math.floor(dataseries.min()))
            # high_lim = int(math.ceil(dataseries.max()))
            # ax.set_xlim([low_lim - stdev, high_lim + stdev])
            # ax.xaxis.set_ticks([low_lim, high_lim])
            ax.set_xlim([10, 450])
            ax.set_ylim([0, num_points])
            ax.xaxis.set_ticks([450, 10])

            ax.set_title("{}".format(16-plot_num), fontsize=18, fontweight="bold")

        for ax in axes_array:
            plt.setp(ax.get_xticklabels(), rotation=90, fontsize=16)

        for ax in axes_array[1:]:
            plt.setp(ax.get_yticklabels(), visible=False)

        axes_array[7].set_xlabel("             Distance", fontsize=20, fontweight="bold", labelpad=50)
        axes_array[0].set_ylabel("Time (seconds ago)", fontsize=20, fontweight="bold", labelpad=50)
        axes_array[0].yaxis.set_ticks([0, num_points/2, num_points])
        # axes_array[0].set_yticklabels([-num_points/sensor_freq/3, -num_points/sensor_freq/2/3, 0])
        axes_array[0].set_yticklabels([-num_points/sensor_freq, -num_points/sensor_freq/2, 0])
        plt.setp(axes_array[0].get_yticklabels(), fontsize=16)

        animation.TimedAnimation.__init__(self, fig, interval=1000 // update_freq, blit=False)

        # start thread for each axis, need to pass it a shared array, segment number

    def _draw_frame(self, framedata):
        # for i, line in enumerate(self.lines):
        #     for _ in range(50 // update_freq):
        #         datastreams[i].append(data[i].next())
        #     line.set_xdata(datastreams[i])

        while not self.q.empty():
            new_data = self.q.get()
            for segment, data in enumerate(new_data):
                # print data
                datastreams[segment].append(data)
        
        for segment, new_data in enumerate(datastreams):
            self.lines[segment].set_xdata(new_data)

        self._drawn_artists = self.lines
        # print time.time()

    def new_frame_seq(self):
        return iter(range(num_points))

    def _init_draw(self):
        for i, line in enumerate(self.lines):
            line.set_xdata([datastreams[i]])


# ani = SubplotAnimation("/dev/tty.usbmodem1411")
ani = SubplotAnimation("/dev/tty.usbmodem1451")
# ani = SubplotAnimation("/dev/tty.SLAB_USBtoUART")
# ani = SubplotAnimation("/dev/tty.usbserial-DA01MEGZ")
ani.data_stream.start()
#ani.save('test_sub.mp4')
plt.show()