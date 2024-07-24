#! /usr/bin/python3

import os
import numpy as np
from matplotlib import pyplot as plt
from argparse import ArgumentParser
from utils import read_csv, read_timestamps, detect_methods

"""
Plot reconstruction metrics for different thresholds and different ground-truths.
"""

# Params:
# Experiment data directory
EXP_DATA_DIR = "/mnt/c/Users/DerFu/Documents/khronos/data/strandmon_gt"

# Auto-generated params:
OUTPUT_DEST = EXP_DATA_DIR + "/plots"


def main():
    print(f"Reading reconciliation data for '{EXP_DATA_DIR}'.")
    # data[name][metric]=values. name=<change>_<reconciler>.
    data = parse_reconciliation_data(EXP_DATA_DIR)
    if len(data) == 0:
        print(f"No reconciliation data found for '{EXP_DATA_DIR}'.")
        return
    compute_mesh_memory(data)

    # Metrics to evaluate.
    metrics = [
        "Accuracy@0.1",
        "Completeness@0.1",
        "RMSE@0.1",
        "MAD@0.1",
        "Chamfer@0.1",
        "Runtime",
        "MeshMemory",
    ]
    metric_names = [
        "Accuracy [%]",
        "Completeness [%]",
        "RMSE [cm]",
        "MAD [cm]",
        "Chamfer [cm]",
        "Runtime [ms]",
        "Memory [MB]",
    ]
    conversion_factors = [100, 100, 100, 100, 100, 1e3, 1e-6]

    # Reconcilers to evaluate.
    names = [k for k in data.keys()]
    method_labels = None  # Optional, otherwise set None.
    # [
    #     "Change Detection",
    #     "No Reconciliation",
    #     "Overwrite",
    # ]

    # Get timestamps
    x = read_timestamps(EXP_DATA_DIR)
    x = (x - x[0]) / 1e9

    # Plot
    plot_overview(
        data, metrics, metric_names, conversion_factors, names, x, method_labels
    )
    plot_comparison(
        data, metrics, metric_names, conversion_factors, names, x, method_labels
    )


def plot_comparison(
    data, metrics, metric_names, conversion_factors, names, x, method_labels
):
    print(f"Plotting reconciliation comparison.")
    if method_labels is None:
        method_labels = names
    plt.figure(figsize=(12, 12))
    n_cols = int(np.ceil(np.sqrt(len(metrics))))
    n_rows = int(np.ceil(len(metrics) / n_cols))
    for i, m in enumerate(metrics):
        plt.subplot(n_rows, n_cols, i + 1)
        for n in names:
            plt.plot(x, data[n][m] * conversion_factors[i])
        plt.title(metric_names[i])
        plt.xlabel("Experiment Time [s]")
        plt.ylabel(metric_names[i])
        if i == 0:
            plt.legend(method_labels)
    plt.tight_layout()

    # Save.
    file_name = os.path.join(OUTPUT_DEST, "reconciliation_comparison.png")
    print(f"Saving plot to '{file_name}'.")
    if not os.path.isdir(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=600)
    print("Done.")


def plot_overview(
    data, metrics, metric_names, conversion_factors, names, x, method_labels
):
    print(f"Plotting reconciliation overview.")
    plt.figure(figsize=(20, 15))
    if method_labels is None:
        method_labels = names
    for i, m in enumerate(metrics):
        y_lower = np.inf
        y_upper = -np.inf
        for n in names:
            y_lower = min(y_lower, np.min(data[n][m]))
            y_upper = max(y_upper, np.max(data[n][m]))
        diff = y_upper - y_lower
        y_lower = (y_lower - 0.05 * diff) * conversion_factors[i]
        y_upper = (y_upper + 0.05 * diff) * conversion_factors[i]

        for j, n in enumerate(names):
            plt.subplot(len(metrics), len(names), i * len(names) + j + 1)
            # plt.plot(
            #     x, data["none"][m] * conversion_factors[i], linestyle="--", color="gray"
            # )
            plt.plot(x, data[n][m] * conversion_factors[i], "b-")
            plt.ylim(y_lower, y_upper)
            if i == 0:
                plt.title(f"{method_labels[j]}")
            if i == len(metrics) - 1:
                plt.xlabel("Experiment Time [s]")
            if j == 0:
                plt.ylabel(metric_names[i])
            if i == 0 and j == 0:
                plt.legend(["Input DSG", "Reconciled DSG"])
    plt.tight_layout()

    # Save.
    file_name = os.path.join(OUTPUT_DEST, "reconciliation_overview.png")
    print(f"Saving plot to '{file_name}'.")
    if not os.path.isdir(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=600)
    print("Done.")


def compute_mesh_memory(data):
    # Add a metric for the memory consumption of the mesh.
    bytes_per_face = 3 * 4
    bytes_per_vertex = 7 * 4
    for d in data.values():
        d["MeshMemory"] = (
            d["FacesAfter"] * bytes_per_face + d["VerticesAfter"] * bytes_per_vertex
        )


def parse_reconciliation_data(exp_data_dir):
    # Add all metrics to a single data struct. method= <change>_<reconciler>.
    # data[method][metric]=values
    data = {}

    # Get all reconcilers.
    for change, rec in detect_methods(exp_data_dir):
        method = change + "_" + rec
        method_dir = os.path.join(exp_data_dir, "pipeline", change, rec)

        # Read mesh data.
        data_file_name = os.path.join(
            exp_data_dir, "results", change, rec, "background_mesh.csv"
        )
        if not os.path.exists(data_file_name):
            print(f"File '{data_file_name}' not found, skipping.")
            continue
        data[method] = read_csv(data_file_name)

        # Read mesh sizes.
        if os.path.isfile(os.path.join(method_dir, "reconciliation_data.csv")):
            size_data = read_csv(os.path.join(method_dir, "reconciliation_data.csv"))
            for key in ["FacesBefore", "VerticesBefore", "FacesAfter", "VerticesAfter"]:
                data[method][key] = size_data[key]
            # Read processing times.
            timing_files = [
                f
                for f in os.listdir(method_dir)
                if f.endswith("_reconciliation_timing.csv")
            ]
            timing_files.sort()
            data[method]["Runtime"] = np.zeros(len(timing_files))
            for i, f in enumerate(timing_files):
                timing_data = read_csv(os.path.join(method_dir, f), by_row=True)
                data[method]["Runtime"][i] = timing_data["reconcile/all"]["mean[s]"]
        else:
            print(
                f"File '{data_file_name}' not found, size and runtime data will be ignored."
            )
            for key in [
                "FacesBefore",
                "VerticesBefore",
                "FacesAfter",
                "VerticesAfter",
                "Runtime",
            ]:
                data[method][key] = np.zeros(len(data[method]["NumGTFailed"]))
    return data


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-e", "--experiment_dir")
    args = parser.parse_args()
    if args.experiment_dir:
        EXP_DATA_DIR = args.experiment_dir
        OUTPUT_DEST = EXP_DATA_DIR + "/plots"
    main()
