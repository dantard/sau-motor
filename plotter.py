#!/usr/bin/env python3
import argparse
import os
import socket
import statistics
import struct
import sys
import time
import signal
from statistics import mean, median

import numpy as np
import yaml
from matplotlib import pyplot as plt
from numpy import arange, std
import matplotlib.pyplot as mpl

mpl.rcParams['font.size'] = 12
mpl.rcParams["font.family"] = "monospace"

parser = argparse.ArgumentParser(
    prog='Delay Watcher',
    description='Whatches Delays!')
parser.add_argument('-a', '--yaml', action='store_true')
parser.add_argument('-f', '--files', default=None, nargs='+')
parser.add_argument('-t', '--type', default="plot", type=str)
parser.add_argument('-d', '--discard', default=0, type=float)
parser.add_argument('-c', '--cut', default=1e20, type=float)
parser.add_argument('-P', '--print', action='store_true')
parser.add_argument('-s', '--sigma', default=3, type=float)
parser.add_argument('-x', '--xticks', default=None, type=str)
parser.add_argument('-y', '--yticks', default=None, type=str)
parser.add_argument('-l', '--legend', default=None, type=str)
parser.add_argument('-X', '--xlabel', default="Time (s)", type=str)
parser.add_argument('-Y', '--ylabel', default="Delay (s)", type=str)

args = parser.parse_args()

for file in args.files:
    if args.yaml:
        with open(file + ".yaml", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            for k in config:
                setattr(args, k, config[k])

    with open(file, "r") as f:
        data = []
        t = []
        for line in f:
            t_, data_ = line.split(",")
            t.append(float(t_))
            data.append(float(data_))

        t = [x - t[0] for x in t]

        data = list(zip(t, data))
        data = [d for d in data if args.discard < d[0] < args.discard + args.cut]
        t, data = zip(*data)

        t = [x - t[0] for x in t]

        print("Mean {:.4f}, Std {:.4f}, Median {:.4f}, Max {:.4f}, Min {:.4f}".format(mean(data), std(data), median(data), max(data), min(data)))

        if args.type == "plot":
            plt.scatter(t, data, 4, c='r', zorder=2)
            plt.plot(t, data, zorder=1)
            plt.xticks(arange(0, max(t), 0.5))
            plt.xlabel("Time (s)")
            plt.ylabel("Delay (s)")

        elif args.type == "hist":
            sigma = statistics.stdev(data)
            mean = statistics.mean(data)
            minimum = max(0, min(data))
            minimum = max(minimum, mean - args.sigma * sigma)
            increment = ((mean + args.sigma * sigma) - minimum) / 250
            bins = np.arange(minimum, mean + args.sigma * sigma, increment)
            plt.hist(data, bins=bins, edgecolor='black', alpha=0.7)

        plt.xlabel(args.xlabel)
        plt.ylabel(args.ylabel)

        if args.legend is not None:
            plt.legend([x for x in args.legend.split(",")], loc='upper right')
        if args.xticks is not None:
            plt.xticks([float(x) for x in args.xticks.split(",")])
        if args.yticks is not None:
            plt.yticks([float(x) for x in args.yticks.split(",")])

        f = mpl.gcf()
        f.set_size_inches(800. / f.dpi, 420. / f.dpi)

if args.print:
    pdf_filename = args.files[0].replace(".csv", ".pdf")
    plt.savefig(pdf_filename, format="pdf", bbox_inches="tight")
    print("Saved to", pdf_filename)
    os.system("pdfcrop " + pdf_filename + " " + pdf_filename)
    os.system("evince " + pdf_filename)
else:
    plt.show()
