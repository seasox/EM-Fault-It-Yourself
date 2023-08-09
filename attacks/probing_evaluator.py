import pickle
import sys
from typing import Optional, Dict, Any

from attacks.probing import Datapoint
from attacks.probing import Metric


def usage():
    print(f"Usage: {sys.argv[0]} $pickle [$board_overlay]")


def discrete_cmap(N, base_cmap=None):
    """Create an N-bin discrete colormap from the specified input map"""

    # Note that if base_cmap is a string or None, you can simply do
    #    return plt.cm.get_cmap(base_cmap, N)
    # The following works for string, None, or a colormap instance:

    from matplotlib import pyplot as plt
    import numpy as np
    base = plt.colormaps[base_cmap]
    color_list = base(np.linspace(0, 1, N))
    cmap_name = base.name + str(N)
    return base.from_list(cmap_name, color_list, N)


def main():
    if len(sys.argv) < 2:
        usage()
        exit(1)
    fname = sys.argv[1]
    overlay = sys.argv[2] if len(sys.argv) > 2 else None
    with open(fname, "rb") as fp:
        def max_cb(values):
            return max(values or [0])

        def success_rate_cb(values):
            return len(list(filter(lambda p: p > 0, values))) / len(values)

        def crash_rate_cb(values):
            return len(list(filter(lambda p: p < 0, values))) / len(values)

        dps: [Datapoint] = pickle.load(fp)  # TODO implement JSON import/export. Pickles are kinda slow
        make_heatmap(dps, overlay, discrete_cmap(10, 'Greys'), Metric.AnyFlipAnywhere, "Max Score", max_cb)
        make_heatmap(dps, overlay, discrete_cmap(10, 'Greens'), Metric.AnyFlipAnywhere, "Pr[#flip > 0]",
                     success_rate_cb)
        make_heatmap(dps, overlay, discrete_cmap(10, 'Reds'), Metric.AnyFlipAnywhere, "Pr[crash]", crash_rate_cb)


def make_heatmap(dps: [[[Datapoint]]], overlay, cmap, metric, title, callback):
    dx = len(dps)
    dy = len(dps[0])
    dz = len(dps[0][0])
    perf_xy_planes = []
    for z in range(dz):
        vis = {}
        perf = [[0.0 for _ in range(dx)] for _ in range(dy)]
        from attacks.probing import evaluate
        for x in range(dx):
            for y in range(dy):
                # the performances of the datapoints created at the location
                values = [evaluate(d, metric) for d in dps[x][y][z]]
                perf[y][x] = callback(values)
        vis["perf"] = perf
        vis["title"] = f"{title}; {metric}; Layer {83 + dps[0][0][z][0].attack_location[2]}"
        perf_xy_planes.append(vis)
    for plane in perf_xy_planes:
        visualize(plane, overlay, cmap)


def visualize(vis: Dict[str, Any], overlay: Optional[str], cmap):
    import matplotlib.pyplot as plt
    import numpy as np
    matrix = np.array(vis["perf"])
    fig, ax = plt.subplots()
    ax.xaxis.tick_top()
    if overlay:
        im = plt.imread(overlay)
        im = np.flipud(im)
        ax.imshow(im, extent=[-1, matrix.shape[1], -1, matrix.shape[0]])
    heatmap = ax.imshow(matrix, cmap=cmap, alpha=0.5, interpolation='nearest')
    ax.set_aspect('equal')
    plt.title(vis["title"])
    fig.colorbar(heatmap)
    plt.show()


if __name__ == '__main__':
    main()
