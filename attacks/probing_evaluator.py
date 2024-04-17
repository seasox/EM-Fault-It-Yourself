import json
import sys
import typing as t
from typing import Optional, Dict, Any

import numpy as np

from attacks.Comm import DatapointDecoder
from attacks.probing import Datapoint
from attacks.probing import Metric


def usage():
    print(f"Usage: {sys.argv[0]} $json [$board_overlay]")


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


def max_cb(values):
    return max(values or [0])


def avg_cb(values):
    non_crash = [value for value in values if value >= 0] or [0]
    return sum(non_crash) / len(non_crash) if len(non_crash) > 0 else 0


def success_rate_cb(values):
    return len(list(filter(lambda p: p > 0, values))) / len(values) if len(values) > 0 else 0


def sub_tuples(a, b) -> t.Tuple:
    assert (len(a) == len(b))
    return tuple([int(a[i] - b[i]) for i in range(len(a))])


def main():
    if len(sys.argv) < 2:
        usage()
        exit(1)
    fname = sys.argv[1]
    overlay = sys.argv[2] if len(sys.argv) > 2 else None

    with open(fname, "r") as fp:
        experiment: dict[str] = json.load(fp, cls=DatapointDecoder)
    if 'settings' in experiment:
        # prepare DP array with offsets
        start = experiment['settings']['experiment']['start']
        start_offset = experiment['settings']['experiment']['start_offset']
        end = experiment['settings']['experiment']['end']
        end_offset = experiment['settings']['experiment']['end_offset']
        start = sub_tuples(start, start_offset)
        end = sub_tuples(end, end_offset)
        dims = sub_tuples(end, start)
        reps = experiment['settings']['experiment']['repetitions'] if 'repetitions' in experiment['settings'][
            'experiment'] else 1000
        dims = tuple([int(d) for d in dims] + [reps])
        dims = sub_tuples(dims, (-1, -1, -1, 0))
        dps = np.empty(dims, dtype=Datapoint)
        start_x, start_y, start_z = start_offset
        measurements = np.array(experiment['datapoints'])
        shape = measurements.shape
        end_x = start_x + shape[0]
        end_y = start_y + shape[1]
        end_z = start_z + shape[2]
        dps[start_x:end_x + 1, start_y:end_y, start_z:end_z + 1] = measurements  # TODO off-by-one?
    else:
        dps: [[[Datapoint]]] = experiment
    make_heatmap(dps, overlay, discrete_cmap(10, 'Greys'),
                 Metric.AnyFlipAnywhere, "Any Flip Anywhere Average Score", avg_cb)
    make_heatmap(dps, overlay, discrete_cmap(10, 'Greens'), Metric.ZeroOneFlipOnR4OrR5, "Pr[0-1 R4 R5 > 0]",
                 success_rate_cb)
    make_heatmap(dps, overlay, discrete_cmap(10, 'Blues'), Metric.ZeroOneFlipOnR4OrR5, "Max(0-1 R4 R5)",
                 max_cb)
    make_heatmap(dps, overlay, discrete_cmap(10, 'Greens'),
                 Metric.ZeroOneFlipOnR4OrR5, "Zero-To-One on R4 or R5", avg_cb)
    # make_heatmap(dps, overlay, discrete_cmap(10, 'Greens'), Metric.ZeroOneFlipOnR4OrR5,
    #             "Max Score for Zero-To-One on R4 or R5",
    #             max_cb)
    make_heatmap(dps, overlay, discrete_cmap(10, 'Reds'),
                 Metric.Crash, "Pr[Crash = 1]", success_rate_cb)
    # make_heatmap(dps, overlay, discrete_cmap(10, 'Reds'), Metric.ResetUnsuccessful, "Reset Unsuccessful",
    #             success_rate_cb)


def make_heatmap(dps: np.typing.NDArray[Datapoint], overlay, cmap, metric, title, callback):
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
                if isinstance(dps[x][y][z], Datapoint):
                    values = [evaluate(d, metric) for d in dps[x][y][z]]
                    perf[y][x] = callback(values)
                else:
                    perf[y][x] = 0
        vis["perf"] = perf
        vis["title"] = f"{title}; {metric}; Z={z}"
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
    if "title" in vis:
        plt.title(vis["title"])
    fig.colorbar(heatmap)
    plt.show()


if __name__ == '__main__':
    main()
