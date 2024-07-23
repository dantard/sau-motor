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
parser.add_argument('-P', '--print', nargs='*')
parser.add_argument('-s', '--sigma', default=3, type=float)
parser.add_argument('-x', '--xticks', default=None, type=str)
parser.add_argument('-y', '--yticks', default=None, type=str)
parser.add_argument('-l', '--legend', default=None, type=str)
parser.add_argument('-X', '--xlabel', default="Time (s)", type=str)
parser.add_argument('-Y', '--ylabel', default="Delay (s)", type=str)
parser.add_argument('-B', '--box', type=str, default="UR1", help="Legend box position")
parser.add_argument('-L', '--line', action='store_false')
parser.add_argument('-2', '--secondaxis', action='store_true')

args = parser.parse_args()

min_ts = 1e20
for file in args.files:
    df = pandas.read_csv(file, header=None)
    min_ts = df.iloc[0, 0] if df.iloc[0, 0] < min_ts else min_ts

file_colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']

label_id = 0
legends = args.legend.split(",") if args.legend is not None else []
legends = [a if a != "" else None for a in legends]
legends.extend([None] * 20)
print(legends)

fig, ax1 = plt.subplots()

for file_id, file in enumerate(args.files):
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

        ax1.scatter(t, data, 6, c=file_colors[file_id], zorder=2, label=legends[label_id])
        label_id = label_id + 1

        ax1.plot(t, data, zorder=1, label=legends[label_id], linewidth=1)
        # plt.xticks(arange(0, max(t), 0.5))
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Delay (s)")

        if df.shape[1] > 2 and args.secondaxis:
            data2 = list(df.iloc[:, 2])
            ax2 = ax1.twinx()
            ax2.plot(t, data2, zorder=1, label=legends[label_id], linewidth=1, color='tab:red')
            ax2.tick_params(axis='y', labelcolor='tab:red')

        label_id = label_id + 1

    elif args.type == "hist":
        data = list(df.iloc[:, 1])
        if args.sigma > 0:
            sigma = statistics.stdev(data)
            mean = statistics.mean(data)
            minimum = max(0, min(data))
            minimum = max(minimum, mean - args.sigma * sigma)
            increment = ((mean + args.sigma * sigma) - minimum) / 250
            bins = np.arange(minimum, mean + args.sigma * sigma, increment)
        else:
            bins = np.arange(min(data), max(data), 0.001)

        counts, bin, patches = ax1.hist(data, bins=bins, edgecolor='black', alpha=0.7, label=legends[label_id])
        sumx = counts.sum()
        counts = list(counts)
        counts.sort(reverse=True)
        ax1.set_yticks(counts[0:4])
        ax1.set_yticklabels(["{:.3f}".format(c / sumx) for c in counts[0:4]])
        '''
        max_valu = max(counts)
        max_perc = max(counts) / counts.sum()
        
        ylab = ["{:.3f}".format(perc) for perc in arange(0, max_perc + 1, max_perc / 5)]
        print(ylab)
        ax1.set_yticklabels(ylab)
        ax1.set_yticks(arange(0, max_valu + 1, max_valu / 5))
        '''
        label_id = label_id + 1

    elif args.type == "packets":
        colors = [['r', 'm'], ['c', 'r'], ['b', 'g']]
        alphas = [[1, 1], [1, 1], [1, 1]]
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
                ax1.scatter(t1, fragment, sizes[j], c=colors[i][j], alpha=alphas[i][j], label=legends[label_id])
                label_id = label_id + 1

        fragment = list(df.iloc[:, 3])
        if args.line:
            ax1.plot(t, fragment, zorder=0, linewidth=1.0, color='#999999', label=legends[label_id])
            label_id = label_id + 1

    ax1.set_xlabel(args.xlabel)
    ax1.set_ylabel(args.ylabel)

    if args.legend is not None:
        where, x = get_legend(args.box)
        ax1.legend(loc=where, bbox_to_anchor=(x, 1))
    if args.xticks is not None:
        xt = args.xticks.split(":")
        print(xt)
        if len(xt) == 1:
            plt.xticks([float(x) for x in xt[0].split(",")])
        else:
            plt.xticks([float(x) for x in xt[0].split(",")], [x for x in xt[1].split(",")])
        # plt.xticks([float(x) for x in args.xticks.split(",")], ["a", "b"])
    if args.yticks is not None:
        ax1.set_yticks([float(x) for x in args.yticks.split(",")])

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
print(args.print, "ooooo")
if args.print is None:
    plt.show()
    sys.exit(0)

# If -P has a value, save the plot that filename
# otherwise save the plot with the same name as the first file
if args.print:
    pdf_filename = args.print[0]
else:
    pdf_filename = args.files[0].replace(".csv", ".pdf")

plt.savefig(pdf_filename, format="pdf", bbox_inches="tight")
print("Saved to", pdf_filename)
os.system("pdfcrop " + pdf_filename + " " + pdf_filename)
os.system("evince " + pdf_filename)
