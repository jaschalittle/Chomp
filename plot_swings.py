from __future__ import division
import itertools
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
import numpy as np
# import pandas as pd
import serial
import sys
import time


def polyfit_derivative(data, num_points, timestep):
    poly_derivatives = list()
    for i in range(len(data) - num_points):
        x = sum(range(num_points)) / num_points
        polyfit = np.polyfit(range(num_points), data[i:i + num_points], 2)
        first_derivative = 2 * polyfit[0] * x + polyfit[1]
        poly_derivatives.append(first_derivative * (1000000 / timestep))
    return np.array(poly_derivatives)

def plot_swing(angles, pressures, throw_close_time, vent_open_time, throw_close_crank_angle, swing_num, filetag, timestep = 1000):
    # write data to log
    with open("{}_swing{}.txt".format(filetag, swing_num), 'w') as fh:
        for angle, pressure in itertools.izip(angles, pressures):
            print >>fh, "{}\t{}".format(angle, pressure)
        print >>fh, "timestep\t{}".format(timestep)
        print >>fh, "throw_close_time\t{}".format(throw_close_time)
        print >>fh, "vent_open_time\t{}".format(vent_open_time)

    # set up x and y for plotting
    angles = np.array(angles)
    pressures = np.array(pressures)
    graph_timestep = timestep / 1000
    x = np.arange(0, len(angles) * graph_timestep, graph_timestep)

    # make plots
    fig = plt.figure()
    ax1 = plt.gca()
    ax1.plot(x, angles)
    plt.ylabel("angle from start (degrees)", color="blue")
    plt.xlabel("time (ms)")
    # ax1 = plt.gca()
    ax2 = ax1.twinx()
    ax2.plot(x, pressures, color="red")
    plt.ylabel("pressure (psi)", color="red")

    # calculate peak velocity and ke in hammer head
    angular_velocities = polyfit_derivative(angles, 13, timestep)
    max_v = np.array(angular_velocities).max() / 360 * 1.07 * 2 * np.pi
    ke = max_v ** 2 * 3.4 / 2   # 3.4 kg, 1.07 m radius arc
    plt.title("{} crank angle fill\npeak {:.2f} m/s\n{:.2f} J".format(throw_close_crank_angle, max_v, ke))

    # adjust axis limits
    ymin1, ymax1 = ax1.get_ylim()
    ymin2, ymax2 = ax2.get_ylim()
    # ymin = min(ymin1, ymin2)
    # ymax = max(ymax1, ymax2)
    ax1.set_ylim(ymin1, ymax1)
    ax2.set_ylim(ymin2, ymax2)

    # valve action labels
    ylabelpos = -15
    ax1.axvline(throw_close_time * graph_timestep, linestyle="dashed", color="black")
    ax1.axvline(vent_open_time * graph_timestep, linestyle="dashed", color="black")
    ax1.text(0, ylabelpos,'throw valve open', rotation=90, ha="center")
    ax1.text(throw_close_time * graph_timestep, ylabelpos,'throw valve close', rotation=90, ha="center")
    ax1.text(vent_open_time * graph_timestep, ylabelpos,'vent valve open', rotation=90, ha="center")

    # adjust major tick location
    ax1.yaxis.set_ticks(np.arange(0, angles.max() + 15, 15))

    # adjust gridlines
    ax1.yaxis.grid(b=True, which='major')
    ax1.xaxis.grid(True, which="both")
    minorLocator = AutoMinorLocator(10)
    ax1.xaxis.set_minor_locator(minorLocator)
    fig.patch.set_facecolor('white')  # make plot surround opaque
    plt.savefig("{}_swing{}.pdf".format(filetag, swing_num), format="pdf", transparent=False, frameon=False, bbox_inches='tight')


def stream_data(serial_device, baudrate):
    ser = serial.Serial(serial_device, baudrate)
    angles = list()
    pressures = list()
    filetag = time.strftime("%Y-%m-%d_%H-%M-%S")
    swing_num = 0
    throw_close_crank_angle = 0;
    while True:
        try:
            line = ser.readline().rstrip().split("\t")
            if line[0] == "timestep":
                timestep = int(line[1])
                print "\t".join(line)
            elif line[0] == "throw_close_timestep":
                throw_close_time = int(line[1])
                print "\t".join(line)
            elif line[0] == "vent_open_timestep":
                vent_open_time = int(line[1])
                print "\t".join(line)
                swing_num += 1
                plot_swing(angles, pressures, throw_close_time, vent_open_time, throw_close_crank_angle, swing_num, filetag, timestep=timestep)
                angles = list()
                pressures = list()
            elif line[0] == "millis":
                if int(line[1]) % 1000 == 0:
                    print "{} seconds".format(int(line[1]) // 1000)
            elif line[0] == "data":
                angle, pressure = map(int, line[1:3])
                angles.append(angle)
                pressures.append(pressure)
            elif line[0] == "throw close crank angle":
                throw_close_crank_angle = line[1];
        except ValueError as err:
            print "\t".join(line)
            print err

def main():
    try:
        serial_device, baudrate = sys.argv[1:]
        baudrate = int(baudrate)
        assert baudrate in [9600, 14400, 19200, 28800, 38400, 56700, 115200], "Expected a common baudrate between 9600 and 115200, inclusive"
    except IndexError:
        print "Should be called as follows:\nplot_swings.py <serial device> <baudrate>"
    except ValueError:
        print "Should be called as follows:\nplot_swings.py <serial device> <baudrate>"
    stream_data(serial_device, baudrate)
    # plt.show()

if __name__ == '__main__':
    main()