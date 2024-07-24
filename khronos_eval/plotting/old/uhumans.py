#!/usr/bin/env python

import os
from matplotlib import pyplot as plt
import csv
import numpy as np

TARGET_DIR = "/mnt/c/Users/DerFu/Documents/khronos/data/test_data"
DPI = 300
DATA_FILE_NAME = "motion_uhumans_10m.csv"


# Plot
def plot(target_dir):
    # Check specified data is valid.
    if not os.path.isdir(target_dir):
        print(f"Target directory '{target_dir}' does not exist.")
        return
    output_dir = os.path.join(target_dir, "plots")
    if not os.path.isdir(output_dir):
        os.mkdir(output_dir)
        print(f"Created output directory '{output_dir}'")

    # Plot detection metrics.
    data_file = os.path.join(target_dir, "motion_detection", DATA_FILE_NAME)
    if os.path.isfile(data_file):
        plot_metrics(data_file, output_dir)
    else:
        print(f"Could motion detection data file '{data_file}' does not exist.")

    # TODO(lschmid): plot detailed timing if we want to.


def plot_metrics(data_file, output_dir):
    print(f"Plotting timing summary from '{data_file}'.")

    _, data = read_csv(data_file)

    # X axis as elapsed time in seconds.
    x = data["Timestamp"]
    x = (x - np.min(x)) / 1e9
    tp = data["True Positives"]
    fp = data["False Positives"]
    fn = data["False Negatives"]
    x_ticks = np.arange(0, np.floor(np.max(x) / 5) * 5, 10)
    num_gt_pixels = tp + fn
    present = np.ones_like(num_gt_pixels)
    v_lines = []
    for p in range(len(num_gt_pixels) - 2):
        if np.sum(num_gt_pixels[p : p + 3]) == 0:
            present[p + 1] = 0
        if present[p] != present[p + 1]:
            v_lines.append(x[p])

    # Plot main categories.
    plt.rcParams.update({"font.size": 12})
    plt.figure(figsize=(12, 12))

    # Plot GT occurence.
    plt.subplot(6, 1, 1)
    for v in v_lines:
        plt.axvline(v, color="gray", alpha=0.2)
    plt.plot(x, num_gt_pixels, color="r")
    num_gt_pixels[present == 0] = np.nan
    plt.plot(x, num_gt_pixels, color="b")
    plt.ylabel("Human Pixels [1]")
    plt.xticks(x_ticks, [])
    plt.xlim(0, np.max(x))

    # Plot Detection Metrics.
    plt.subplot(6, 1, 2)
    for v in v_lines:
        plt.axvline(v, color="gray", alpha=0.2)
    norm = tp + fp
    norm[norm == 0] = np.nan
    prec = tp / norm * 100
    plt.plot(x, prec, color="b")
    plt.xlim(0, np.max(x))
    plt.xticks(x_ticks, [])
    plt.ylabel("Precision [%]")
    print(f"Mean Precision: {np.nanmean(prec):.2f}")

    plt.subplot(6, 1, 3)
    for v in v_lines:
        plt.axvline(v, color="gray", alpha=0.2)
    norm = tp + fn
    norm[norm == 0] = np.nan
    rec = tp / norm * 100
    plt.plot(x, rec, color="r")
    plt.xlim(0, np.max(x))
    plt.xticks(x_ticks, [])
    plt.ylabel("Recall [%]")
    print(f"Mean Recall: {np.nanmean(rec):.2f}")

    plt.subplot(6, 1, 4)
    for v in v_lines:
        plt.axvline(v, color="gray", alpha=0.2)
    norm = rec + prec
    norm[norm == 0] = np.nan
    f1 = 2 * rec * prec / norm
    plt.plot(x, f1, color="g")
    plt.xlim(0, np.max(x))
    plt.xticks(x_ticks, [])
    plt.ylabel("F1 Score [%]")
    print(f"Mean F1: {np.nanmean(f1):.2f}")

    plt.subplot(6, 1, 5)
    for v in v_lines:
        plt.axvline(v, color="gray", alpha=0.2)
    norm = tp + fp + fn
    norm[norm == 0] = np.nan
    iou = tp / norm * 100
    plt.plot(x, iou, color="k")
    plt.xlim(0, np.max(x))
    plt.xticks(x_ticks, [])
    plt.ylabel("IoU [%]")
    print(f"Mean IoU: {np.nanmean(iou):.2f}")

    plt.subplot(6, 1, 6)
    for v in v_lines:
        plt.axvline(v, color="gray", alpha=0.2)
    plt.plot(x, data["Num Objects"], color="b")
    plt.xlim(0, np.max(x))
    plt.ylabel("Detected Objects [1]")
    plt.xticks(x_ticks)
    plt.xlabel("Time [s]")
    plt.tight_layout()

    # Save
    file_name = os.path.join(output_dir, "motion_detection.png")
    plt.savefig(file_name, dpi=DPI)
    print(f"Saved motion detection analysis '{file_name}'.")
    return


def read_csv(csv_file):
    if not os.path.isfile(csv_file):
        print(f"File '{csv_file}' does not exist.")
        return
    # Read data.
    data = {}
    with open(csv_file, "r") as read_obj:
        csv_reader = csv.reader(read_obj)
        headers = []
        for row in csv_reader:
            if not headers:
                headers = row
                for h in headers:
                    data[h] = []
            else:
                for i, d in enumerate(row):
                    data[headers[i]].append(float(d))
    for d in data:
        data[d] = np.array(data[d])
    return headers, data


def main():
    plot(TARGET_DIR)


if __name__ == "__main__":
    main()
