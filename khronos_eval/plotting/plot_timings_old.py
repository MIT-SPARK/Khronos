#! /usr/bin/python3

import os
import numpy as np
from matplotlib import pyplot as plt
from argparse import ArgumentParser
from utils import (
    read_csv,
    read_timestamps,
    detect_methods,
)


# Params:
EXP_DATA_DIR = "/mnt/c/Users/DerFu/Documents/khronos/data/khronos/office_kimera"
PLOT_DPI = 300

# Auto-generated params:
OUTPUT_DEST = EXP_DATA_DIR + "/plots"


def main():
    # data[method][metric][map_name][query_time_index]=values
    print(f"Reading timing data for '{EXP_DATA_DIR}'.")
    data = parse_data(EXP_DATA_DIR)

    # NOTE(lschmid): Currently just plot the change detection and reconciliation timings.
    # robot_times =

    if len(data) == 0:
        print(f"No timing data found for '{EXP_DATA_DIR}'.")
        return

    # available keys
    metrics = list(data[list(data.keys())[0]].keys())
    print(metrics)

    # Plot
    plot_details(data, "Middle_Optimistic")
    plot_comparison(data)

    print("Done.")


def plot_details(data, method):
    query_times = [float(x) for x in read_timestamps(EXP_DATA_DIR)]
    x = (np.array(query_times) - query_times[0]) / 1e9

    # Plot.
    plt.figure(figsize=(12, 12))
    plt.subplot(2, 1, 1)
    m1 = ["reconcile/all", "change_detection_recompute/all"]
    plt.title("Overview")
    for m in m1:
        plt.plot(x, data[method][m])
    plt.ylabel("Runtime [s]")
    plt.xlabel("Experiment Time [s]")
    plt.legend(m1)

    plt.subplot(2, 1, 2)
    m2 = [
        "reconcile/objects",
        "change_detection_recompute/objects",
        "change_detection_recompute/update_ray_verificator",
    ]
    plt.title("Change Detection")
    for m in m2:
        plt.plot(x, data[method][m])
    plt.ylabel("Runtime [s]")
    plt.xlabel("Experiment Time [s]")
    plt.legend(["Reconciliation", "Change Detection", "Update Rays (on LC only)"])

    # Save.
    file_name = os.path.join(OUTPUT_DEST, f"Timing_Offline.png")
    print(f"Saving plot to '{file_name}'.")
    if not os.path.isdir(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=600)
    plt.close()


def plot_comparison(data):
    query_times = [float(x) for x in read_timestamps(EXP_DATA_DIR)]
    x = (np.array(query_times) - query_times[0]) / 1e9
    metrics = [
        "change_detection_recompute/all",
        "change_detection_recompute/background",
        "change_detection_recompute/objects",
        "change_detection_recompute/update_ray_verificator",
    ]
    methods = list(data.keys())

    # Plot.
    plt.figure(figsize=(16, 8))
    for i, metric in enumerate(metrics):
        plt.subplot(1, len(metrics), i + 1)
        plt.title(metric)
        for method in methods:
            plt.plot(x, data[method][metric])
        plt.ylabel("Runtime [s]")
        plt.xlabel("Experiment Time [s]")
        if i == 0:
            plt.legend(methods)

    # Save.
    plt.tight_layout()
    file_name = os.path.join(OUTPUT_DEST, f"Timing_Offline_Comp.png")
    print(f"Saving plot to '{file_name}'.")
    if not os.path.isdir(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=600)


def parse_data(exp_data_dir):
    # Add all metrics to a single data struct. method= <change>_<reconciler>.
    # data[method][metric]=values
    data = {}

    # Get all reconcilers.
    for change, rec in detect_methods(exp_data_dir):
        method = change + "_" + rec
        method_dir = os.path.join(exp_data_dir, "pipeline", change, rec)
        data[method] = {}

        # Read reconciliation processing times.
        read_timing_files(method_dir, "_reconciliation_timing.csv", data[method])

        # Read change detection processing times.
        change_dir = os.path.join(exp_data_dir, "pipeline", change)
        read_timing_files(change_dir, "_change_detection_timing.csv", data[method])

    return data


def read_timing_files(directory, file_pattern, data):
    # Read timing data where a file is created for every time step. Adds to the data
    timing_files = [f for f in os.listdir(directory) if f.endswith(file_pattern)]
    timing_files.sort()
    for i, f in enumerate(timing_files):
        timing_data = read_csv(os.path.join(directory, f), by_row=True)
        for key, value in timing_data.items():
            if key not in data:
                data[key] = np.zeros(len(timing_files))
            data[key][i] = value["mean[s]"]


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-e", "--experiment_dir")
    parser.add_argument("-c", "--eval_config")
    args = parser.parse_args()
    if args.experiment_dir:
        EXP_DATA_DIR = args.experiment_dir
        OUTPUT_DEST = EXP_DATA_DIR + "/plots"
    if args.eval_config:
        EVAL_CONFIG = args.eval_config
    main()
