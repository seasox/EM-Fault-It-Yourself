import pickle
import sys

from Comm import STATUS
#from probing import evaluate, Datapoint, Metric
import matplotlib.pyplot as plt

def main():
    with open("/home/pi/pycharm_mnt/EM-Fault-It-Yourself/pickles/data_2023-06-28-14:12:31.pickle", "rb") as fp:
        # TODO this might do stupid stuff use with caution
        dps = pickle.load(fp)
        dx = len(dps)
        dy = len(dps[0])
        dz = len(dps[0][0])
        perf_xy_planes = []
        for z in range(dz):
            perf = [[0 for _ in range(dy)] for _ in range(dx)]
            for x in range(dx):
                for y in range(dy):
                    # the performances of the datapoints created at the location
                    values = [evaluate(d, Metric.AnyFlipAnywhere) for d in dps[x][y][z]]
                    perf[x][y] = max(values)
            perf_xy_planes.append(perf)
        visualize(perf_xy_planes[0])

def visualize(xy_plane):
    plt.imshow(xy_plane, cmap='viridis', interpolation='nearest')
    plt.colorbar()
    plt.show()


if __name__ == '__main__':
    main()
