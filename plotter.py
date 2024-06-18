#!/usr/bin/env python3
import argparse
import os
import re
import statistics
import sys

import matplotlib.pyplot as mpl
import numpy as np
import pandas
import yaml
from matplotlib import pyplot as plt
from numpy import arange

mpl.rcParams['font.size'] = 12
mpl.rcParams["font.family"] = "monospace"


def get_legend(input_string):
    print("aaaaaaaaaaaa", input_string)
    matches = re.match(r"([A-Z]{2})([\d.]*)", input_string.strip(" "))
    if matches:
        prefix, value = matches.groups()
        if not value:  # If the value is an empty string
            value = '1'
        prefix = prefix.replace("UR", "upper right")
        prefix = prefix.replace("UL", "upper left")
        prefix = prefix.replace("LL", "lower left")
        prefix = prefix.replace("LR", "lower right")
        return prefix, float(value)
    else:
        return None, None


parser = argparse.ArgumentParser(
    prog='Plotter',
    description='Plots!')
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
parser.add_argument('-B', '--box', type=str, default="UR1", help="Legend box position")
parser.add_argument('-L', '--line', action='store_false')

args = parser.parse_args()

min_ts = 1e20
for file in args.files:
    df = pandas.read_csv(file, header=None)
    min_ts = df.iloc[0, 0] if df.iloc[0, 0] < min_ts else min_ts

for file in args.files:
    if args.yaml:
        with open(file + ".yaml", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            for k in config:
                setattr(args, k, config[k])

    df = pandas.read_csv(file, header=None)
    df.iloc[:, 0] = df.iloc[:, 0] - min_ts  # - df.iloc[0, 0]

    df = df[df.iloc[:, 0] > args.discard]
    df = df[df.iloc[:, 0] < args.discard + args.cut]
    df.iloc[:, 0] = df.iloc[:, 0] - args.discard
    t = list(df.iloc[:, 0])

    if args.type == "plot":
        data = list(df.iloc[:, 1])
        plt.scatter(t, data, 4, c='r', zorder=2)
        plt.plot(t, data, zorder=1)
        # plt.xticks(arange(0, max(t), 0.5))
        plt.xlabel("Time (s)")
        plt.ylabel("Delay (s)")

    elif args.type == "hist":
        data = list(df.iloc[:, 1])
        sigma = statistics.stdev(data)
        mean = statistics.mean(data)
        minimum = max(0, min(data))
        minimum = max(minimum, mean - args.sigma * sigma)
        increment = ((mean + args.sigma * sigma) - minimum) / 250
        bins = np.arange(minimum, mean + args.sigma * sigma, increment)
        plt.hist(data, bins=bins, edgecolor='black', alpha=0.7)

    elif args.type == "packets":
        colors = [['r', 'm'], ['c', 'r'], ['b', 'g']]
        alphas = [[0.5, 1], [1, 0.25], [1, 1]]
        zorder = [1, 2]
        sizes = [8, 8]

        for i in [1, 2]:
            for j in [0, 1]:
                filtered = df[df.iloc[:, 1] == i]
                filtered = filtered[filtered.iloc[:, 2] % 2 == j]
                t1 = list(filtered.iloc[:, 0])
                index = list(filtered.iloc[:, 1])
                serial = list(filtered.iloc[:, 2])
                fragment = list(filtered.iloc[:, 3])
                cols = [colors[i][s % 2] for s in serial]
                plt.scatter(t1, fragment, sizes[j], c=colors[i][j], alpha=alphas[i][j])

        fragment = list(df.iloc[:, 3])
        if args.line:
            plt.plot(t, fragment, zorder=0, linewidth=1.0, color='#999999')

    plt.xlabel(args.xlabel)
    plt.ylabel(args.ylabel)

    if args.legend is not None:
        where, x = get_legend(args.box)
        plt.legend([x.strip(" ") for x in args.legend.split(",")], loc=where, bbox_to_anchor=(x, 1))
    if args.xticks is not None:
        plt.xticks([float(x) for x in args.xticks.split(",")])
    if args.yticks is not None:
        plt.yticks([float(x) for x in args.yticks.split(",")])

    f = mpl.gcf()
    f.set_size_inches(800. / f.dpi, 420. / f.dpi)


def connect_close(figure=None):
    def press_key(event):
        if event.key == 'escape':
            plt.close('all')
            sys.exit(0)

    if not figure:
        figure = plt.gcf()
    figure.canvas.mpl_connect('key_press_event', press_key)


connect_close(figure=plt.gcf())

if args.print:
    pdf_filename = args.files[0].replace(".csv", ".pdf")
    plt.savefig(pdf_filename, format="pdf", bbox_inches="tight")
    print("Saved to", pdf_filename)
    os.system("pdfcrop " + pdf_filename + " " + pdf_filename)
    os.system("evince " + pdf_filename)
else:
    plt.show()
