#! /usr/bin/python3

import pathlib
import sys
import numpy as np
import argparse
from matplotlib import pyplot as plt
from utils import (
    get_4d_data_slice,
    augment_object_metrics,
    augment_dynamic_metrics,
    augment_bg_metrics,
    read_map_names,
    parse_table_data_simple,
    read_map_timestamps,
)

"""
Plot reconstruction metrics for different thresholds and different ground-truths.
"""

# Params:
PRINT_NAMES = True
PRINT_STD = False
PRINT_NAN = False
PRINT_MODE = "human"  # 'human', 'latex', 'csv'
DATA_SLICE = "4D"  # '4D', 'Robot', 'Query', 'Online'

# Put all metrics you want to evaluate over here.
METRICS = [
    "Accuracy@0.2",
    "Completeness@0.2",
    "F1@0.2",
    "ObjectPrecision",
    "ObjectRecall",
    "ObjectF1",
    "DynamicPrecision",
    "DynamicRecall",
    "DynamicF1",
    "ChangePrecision",
    "ChangeRecall",
    "ChangeF1",
]


def all_methods(include_method=[True, True, True, True]):
    methods = []
    names = []
    rec_names = ["Hydra/Hydra", "Dyna/Blox", "Panoptic/Mapping", "Middle/Optimistic"]
    read_names = ["Hydra", "Dynablox", "Panoptic 67", "Khronos"]
    for scene in ["apartment", "office"]:
        for gt in ["gt", "kimera"]:
            for i, method in enumerate(
                ["hydra", "dynablox", "panoptic_mapping", "khronos"]
            ):
                if not include_method[i]:
                    continue
                methods.append(f"{method}/{scene}_{gt}/results/{rec_names[i]}")
                names.append(f"{scene} - {gt} - {read_names[i]}")
    return methods, names


def main():
    parser = argparse.ArgumentParser(description="Show table of results")
    parser.add_argument("data_dir", help="top-level directory containing results")
    parser.add_argument(
        "-m",
        "--methods",
        nargs="+",
        help="methods to evaluate (optional label via method:label)",
        default=None,
    )
    args = parser.parse_args()

    data_dir = pathlib.Path(args.data_dir).expanduser().absolute()
    if not data_dir.exists():
        print(f"Invalid data_dir: '{data_dir}'")
        sys.exit(1)

    methods = []
    method_labels = []
    if args.methods is None:
        methods.append("tmp")
        method_labels.append("tmp")
    else:
        for x in args.methods:
            parts = x.split(":")
            if len(parts) == 1:
                methods.append(parts[0])
                method_labels.append(parts[0])
            else:
                methods.append(parts[0])
                method_labels.append(parts[1])

    # data[method_index][metric][map_no][timestamp]=value.
    print("Loading data.")
    data = parse_table_data_simple(str(data_dir), methods)

    # Get common data
    timestamps = [read_map_timestamps(str(data_dir / m / "results")) for m in methods]
    map_names = [[int(x) for x in read_map_names(str(data_dir / m))] for m in methods]

    has_dynamic = any([m.startswith("Dynamic") for m in METRICS])
    has_objects = any(
        [m.startswith("Object") or m.startswith("Change") for m in METRICS]
    )
    has_bg = any([m.startswith("F1@") for m in METRICS])
    for d in data:
        if has_objects:
            augment_object_metrics(d)
        if has_dynamic:
            augment_dynamic_metrics(d)
        if has_bg:
            augment_bg_metrics(d)

    # Plot
    table(data, map_names, timestamps, method_labels, METRICS)


def table(data, map_names, timestamps, method_labels, metrics):
    conversion_factors = [100] * len(metrics)

    all_data = {}
    for m in metrics:
        all_data[m] = np.array([])

    print_row((["Data"] + metrics) if PRINT_NAMES else metrics)
    for i, n in enumerate(method_labels):
        entries = [n] if PRINT_NAMES else []
        for mi, m in enumerate(metrics):
            values = (
                extract_data(data[i][m], map_names[i], timestamps[i])
                * conversion_factors[mi]
            )
            all_data[m] = np.append(all_data[m], values)
            entries.append(format_values(values))
        print_row(entries)


def print_row(entries):
    if PRINT_MODE == "latex":
        print(("".join("%-5s & " % x for x in entries))[:-2] + "\\\\")
    elif PRINT_MODE == "csv":
        print(("".join("%s," % x for x in entries))[:-1])
    else:
        print("".join("%-20s" % x for x in entries))


def format_values(results):
    nans = np.sum(np.isnan(results))
    msg = f"{np.nanmean(results):.1f}" if nans < len(results) else "-"
    if PRINT_STD and nans < len(results):
        if PRINT_MODE == "latex":
            msg = msg + f" $\pm$ {np.nanstd(results):.1f}"
        elif PRINT_MODE == "csv":
            msg = msg + f",{np.nanstd(results):.1f}"
        else:
            msg = msg + f" +- {np.nanstd(results):.1f}"

    if PRINT_NAN and nans > 0:
        msg = msg + f" ({nans})"
    return msg


def extract_data(data, map_names, query_times):
    if DATA_SLICE == "4D":
        result = []
        for d in data.values():
            result = result + [x for x in d.values()]
        return np.array(result)
    elif DATA_SLICE == "Robot" or DATA_SLICE == "Query" or DATA_SLICE == "Online":
        return get_4d_data_slice(data, map_names, query_times, DATA_SLICE)
    else:
        print(f"Unknown data slice '{DATA_SLICE}'")
        return np.array([np.nan])


def plot_gt_change_lines(x_values):
    y_min, y_max = plt.gca().get_ylim()
    plt.vlines(x_values, ymin=0, ymax=y_max, colors="gray", alpha=0.3)


if __name__ == "__main__":
    main()
