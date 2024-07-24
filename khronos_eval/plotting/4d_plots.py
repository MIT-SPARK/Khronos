#! /usr/bin/python3

import os
import numpy as np
from matplotlib import pyplot as plt
from utils import (
    parse_table_data,
    read_timestamps,
    augment_object_metrics,
)
from plot_4d import plot_4d


# Params:
DATA_DIR = "/mnt/c/Users/DerFu/Documents/khronos/data"
# METHOD = "jackal/results/Middle/NotMerged"
METHOD = "jackal_lukas/results/Middle/Optimistic"
# METHOD = "khronos/apartment_gt/results/M36/Optimistic"
# METHOD = "panoptic_mapping/office_kimera/results/Panoptic/Mapping"
OUTPUT_DEST = "/mnt/c/Users/DerFu/Documents/khronos/plots/4dmaps/jackal"
METRICS = [
    # "Accuracmy@0.2",
    # "Completeness@0.2",
    # "F1@0.2",
    # "ObjectPrecision",
    # "ObjectRecall",
    # "ObjectF1",
    # "ChangePrecision",
    # "ChangeRecall",
    # "ChangeF1",
    # "ChangePrecisionIncl",
    # "ChangeRecallIncl",
    # "ChangeF1Incl",
    # "ChangeTP",
    # "ChangeFP",
    # "ChangeFN",
    # "ChangeTN",
    "NumObjDetected"
]
QUERY_INDICES = [0.25, 0.5, 0.75]


def main():
    # data[method_index][metric][map_no][timestamp]=value.
    print(f"Loading data.")
    data = parse_table_data(DATA_DIR, [METHOD])[0]
    # print(data.keys())
    exp_dir = os.path.join(DATA_DIR, METHOD.split("/results/")[0])
    timestamps = read_timestamps(exp_dir)
    timestamps = [(t - timestamps[0]) / 1e9 for t in timestamps]
    augment_object_metrics(data)

    # Plot
    for m in METRICS:
        plot(data[m], timestamps, m)


def plot(data, timestamps, metric_name):
    # Get data into grid.
    y = []
    # NOTE(lschmid): This relies on the assumption that the dict values are iterated in the order of creation (which should hold for Python 3.7+)
    for map_data in data.values():
        y.append(list(map_data.values()))

    # Plot
    print(f"Plotting '{metric_name}'.")
    plot_surface = False
    output_file = os.path.join(
        OUTPUT_DEST, f"{metric_name}{'_surf' if plot_surface else ''}"
    )
    os.makedirs(OUTPUT_DEST, exist_ok=True)
    plot_4d(
        timestamps,
        y,
        metric_name,
        output_file_name=output_file,
        query_detail_indices=[round(i * len(timestamps)) for i in QUERY_INDICES],
        plot_surface=plot_surface,
        surface_resolution=500,
        save_intermediate_frames=False,
    )


if __name__ == "__main__":
    main()
