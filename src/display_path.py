import csv
from matplotlib import pyplot as plt
import numpy as np


def main():
    paths_x, paths_y=[], []
    csv_fname = '../data/log.txt'
    with open(csv_fname) as csv_file:
        reader = csv.reader(csv_file)
        for line in reader:
            items = line[0].split()
            paths_x.append(items[:200])
            paths_y.append(items[200:])

    print('Read', len(paths_x), 'lines from input file', csv_fname)

    for x, y in zip(paths_x, paths_y):
        fig, ax = plt.subplots()
        plt.plot(x, y, marker='.')
        plt.axis('equal')
        plt.show()

    """fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.plot(x, y, marker='.')
    xy_pairs = zip(x, y)
    count = 0
    for the_s, xy in zip(s, xy_pairs):
        if count % 10 == 0:
            ax.annotate('{:.0f}'.format(float(the_s)), xy=xy, textcoords='data')
        count += 1
    plt.show()"""

    """txt_fname = '../data/log.txt'
    lines = []
    with open(txt_fname) as txt_file:
        reader = csv.reader(txt_file)
        for line in reader:
            lines.append(line[0]);

    for line in lines:
        car_s, s_start, s_vel_start, s_acc_start, s_goal, s_vel_goal, s_acc_goal, a0, a1, a2, a3, a4, a5 = line.split()
        x = np.linspace(0, 3.92, 199)
        a0 = float(a0)
        a1 = float(a1)
        a2 = float(a2)
        a3 = float(a3)
        a4 = float(a4)
        a5 = float(a5)
        y = a0 + a1 * x + a2 * (x ** 2) + a3 * (x ** 3) + a4 * (x ** 4) + a5 * (x ** 5)
        fig, ax = plt.subplots()
        plt.plot(x, y)
        plt.show()"""


if __name__ == "__main__":
    main()