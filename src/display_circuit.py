import csv
from matplotlib import pyplot as plt


def main():
    x, y, s = [], [], []
    csv_fname = '../data/highway_map.csv'
    with open(csv_fname) as csv_file:
        reader = csv.reader(csv_file)
        for line in reader:
            one_x, one_y, one_s, _, _ = line[0].split(sep=' ')
            x.append(one_x)
            y.append(one_y)
            s.append(one_s)

    print('Read', len(x), 'lines from input csv file', csv_fname)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.plot(x, y, marker='.')
    xy_pairs = zip(x, y)
    count = 0
    for the_s, xy in zip(s, xy_pairs):
        if count % 10 == 0:
            ax.annotate('{:.0f}'.format(float(the_s)), xy=xy, textcoords='data')
        count += 1
    plt.show()


if __name__ == "__main__":
    main()
