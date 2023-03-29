import pickle
from pprint import pprint

from attacks.probing import Datapoint, Metric
import matplotlib.pyplot as plt

def main():
    with open("data.pickle", "rb") as fp:
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
                    values = [d.evaluate(Metric.AnyFlipAnywhere) for d in dps[x][y][z]]
                    perf[x][y] = -1 if -1 in values else sum(values) / len(values)
            perf_xy_planes.append(perf)
        visualize(perf_xy_planes[0])

def visualize(xy_plane):
    plt.imshow(xy_plane, cmap='viridis', interpolation='nearest')
    plt.colorbar()
    plt.show()


if __name__ == '__main__':
    main()
