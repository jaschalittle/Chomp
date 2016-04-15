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

num_points = 1000

# in Hz. should be even divisor of 50 and 1000
update_freq = 1

num_timers = 3

datastreams = [collections.deque([0] * num_points, maxlen=num_points) for _ in range(num_timers)]
# leddar_loop_times = collections.deque(np.nan * num_points, maxlen=num_points)

def stream_data(serial_port, q):
    ser = serial.Serial(serial_port, 115200)
    while True:
        line = ser.readline()
        line = line.rstrip().split("\t")
        try:
            loop_type = int(line[0])
            loop_time = int(line[1])
            q.put([loop_type, loop_time])
            # print loop_type, loop_time
            # loop contained leddar processing
        except ValueError as err:
            print err
            print line
        except IndexError as err:
            print err
            print line
        # print data_to_send
        # q.put(new_data)

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


# This example uses subclassing, but there is no reason that the proper
# function couldn't be set up and then use FuncAnimation. The code is long, but
# not really complex. The length is due solely to the fact that there are a
# total of 9 lines that need to be changed for the animation as well as 3
# subplots that need initial set up.
class SubplotAnimation(animation.TimedAnimation):
    def __init__(self, serial_port):

        # make list of shared array objects here so that another thread can read serial data.
        # if this thread continues plotting data in array, it should keep itself up to date as long as
        # its refresh rate exceeds data streaming rate

        # self.datastreams = [multiprocessing.Array('i', num_points) for _ in 48]
        self.num_bins = 10
        self.q = multiprocessing.Queue()
        self.data_stream = multiprocessing.Process(target=stream_data, args=(serial_port, self.q))

        fig, ax = plt.subplots()
        ax.hist([0] * num_points, bins=self.num_bins, normed=True, color="red")
        ax.hist([0] * num_points, bins=self.num_bins, normed=True, color="green")
        ax.hist([0] * num_points, bins=self.num_bins, normed=True, color="blue")
        self.ax = ax
        fig.suptitle("Loop time", fontweight="bold", fontsize=24)

        # plt.xlim(0, )
        # ax.set_ylim([0, num_points])
        # ax.xaxis.set_ticks([450, 10])

            # ax.set_title("{}".format(16-plot_num), fontsize=18, fontweight="bold")

        # for ax in axes_array:
        #     plt.setp(ax.get_xticklabels(), rotation=90, fontsize=16)

        # for ax in axes_array[1:]:
        #     plt.setp(ax.get_yticklabels(), visible=False)

        # axes_array[7].set_xlabel("             Distance", fontsize=20, fontweight="bold", labelpad=50)
        # axes_array[0].set_ylabel("Time (seconds ago)", fontsize=20, fontweight="bold", labelpad=50)
        # axes_array[0].yaxis.set_ticks([0, num_points/2, num_points])
        # axes_array[0].set_yticklabels([-num_points/sensor_freq/3, -num_points/sensor_freq/2/3, 0])
        # plt.setp(axes_array[0].get_yticklabels(), fontsize=16)

        animation.TimedAnimation.__init__(self, fig, interval=500, blit=False)

        # start thread for each axis, need to pass it a shared array, segment number

    def _draw_frame(self, framedata):
        # for i, line in enumerate(self.lines):
        #     for _ in range(50 // update_freq):
        #         datastreams[i].append(data[i].next())
        #     line.set_xdata(datastreams[i])

        while not self.q.empty():
            new_data = self.q.get()
            if new_data[0] & 2:
                datastreams[0].append(int(new_data[1]))
            # loop contained SBUS processing
            elif new_data[0] & 4:
                datastreams[1].append(int(new_data[1]))
            # all other loops
            else:
                datastreams[2].append(int(new_data[1]))
            # datastream.extend(new_data)
        
        self.ax.cla()
        self.ax.hist(datastreams[0], bins=self.num_bins, normed=True, color="red", edgecolor="none")
        self.ax.hist(datastreams[1], bins=self.num_bins, normed=True, color="green", edgecolor="none")
        self.ax.hist(datastreams[2], bins=self.num_bins, normed=True, color="blue", edgecolor="none")
        # plt.xlim(0, )
        plt.ylim(0, 0.3)
        self._drawn_artists = self.ax
        # print time.time()

    def new_frame_seq(self):
        return iter(range(num_points))

    def _init_draw(self):
        self.ax.hist(datastreams[0], bins=self.num_bins, normed=True, color="red", edgecolor="none")
        self.ax.hist(datastreams[1], bins=self.num_bins, normed=True, color="green", edgecolor="none")
        self.ax.hist(datastreams[2], bins=self.num_bins, normed=True, color="blue", edgecolor="none")


ani = SubplotAnimation("/dev/tty.usbmodem1411")
# ani = SubplotAnimation("/dev/tty.usbmodem1451")
# ani = SubplotAnimation("/dev/tty.SLAB_USBtoUART")
ani.data_stream.start()
#ani.save('test_sub.mp4')
plt.show()