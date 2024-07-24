#! /usr/bin/python3

import os
from matplotlib import pyplot as plt
from utils import read_csv, read_timestamps

"""
Plot reconstruction metrics for different thresholds and different ground-truths.
"""

# Params:
# Experiment data directory
EXP_DATA_DIR = "/mnt/c/Users/DerFu/Documents/khronos/data/mesh_test_data"

# Auto-generated params:
OUTPUT_DEST = EXP_DATA_DIR + "/plots/mesh_metrics.png"


def main():
    print(f"Plotting mesh metrics for '{EXP_DATA_DIR}'...")
    # Load data. data[idx][header] = values. idx = 0: 1000, idx = 1: 10000, idx = 2: mesh
    data = [
        read_csv(EXP_DATA_DIR + "/metrics/mesh_1000.csv"),
        read_csv(EXP_DATA_DIR + "/metrics/mesh_10000.csv"),
        read_csv(EXP_DATA_DIR + "/metrics/mesh_mesh.csv"),
    ]

    metrics = ["Accuracy", "Completeness", "RMSE", "MAD", "Chamfer"]
    units = ["%", "%", "m", "m", "m"]
    thresholds = [0.05, 0.08, 0.1, 0.2, 0.3, 0.5]
    linestyles = ["-", "--", "-."]
    colors = ["b", "k", "g"]

    # Get timestamps
    x = read_timestamps(EXP_DATA_DIR)
    x = (x - x[0]) / 1e9

    # Plot
    plt.figure(figsize=(20, 15))
    for i, m in enumerate(metrics):
        for j, t in enumerate(thresholds):
            plt.subplot(len(metrics), len(thresholds), i * len(thresholds) + j + 1)
            key = f"{m}@{t}"
            for k, d in enumerate(data):
                values = d[key]
                if i < 2:
                    values *= 100
                plt.plot(x, d[key], linestyle=linestyles[k], color=colors[k])
            plt.title(f"{m}@{t}m")

            # Conditional formatting.
            if i < 2:
                plt.ylim([0, 100])
            else:
                plt.ylim([0, 0.3])
            if i == len(metrics) - 1:
                plt.xlabel("Experiment Time [s]")
            if j == 0:
                plt.ylabel(m + " [" + units[i] + "]")
            if i == 0 and j == 0:
                plt.legend(["1000", "10000", "Mesh"])
    plt.tight_layout()

    # Save.
    print(f"Saving plot to '{OUTPUT_DEST}'...")
    if not os.path.exists(os.path.dirname(OUTPUT_DEST)):
        os.makedirs(os.path.dirname(OUTPUT_DEST))
    plt.savefig(OUTPUT_DEST, dpi=600)
    print("Done.")


if __name__ == "__main__":
    main()
