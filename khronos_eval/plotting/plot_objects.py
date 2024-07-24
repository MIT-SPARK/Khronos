#! /usr/bin/python3

import os
import numpy as np
from matplotlib import pyplot as plt
from argparse import ArgumentParser
from utils import (
    read_timestamps,
    read_map_names,
    compute_metrics,
    read_object_data,
    get_4d_data_slice,
    read_gt_changes,
)

"""
Plot reconstruction metrics for different thresholds and different ground-truths.
"""

# Params:
# Experiment data directory
EXP_DATA_DIR = "/mnt/c/Users/DerFu/Documents/khronos/data/khronos/apartment_gt"
PLOT_DPI = 300
EVAL_CONFIG = ""

# Auto-generated params:
OUTPUT_DEST = EXP_DATA_DIR + "/plots"


def main():
    print(f"Reading static object data for '{EXP_DATA_DIR}'.")
    # data[method][metric][map_name][query_time_index]=values
    data = read_object_data(EXP_DATA_DIR)

    if len(data) == 0:
        print(f"No static object data found for '{EXP_DATA_DIR}'.")
        return

    # Read GT changes if supplied.
    gt_appeared_times, gt_disappeared_times = read_gt_changes(EVAL_CONFIG, True)

    # Plot
    # plot(data, gt_appeared_times, gt_disappeared_times, "Robot")
    plot(data, gt_appeared_times, gt_disappeared_times, "Query")
    plot(data, gt_appeared_times, gt_disappeared_times, "Online")

    print("Done.")


def plot(data, gt_appeared_times, gt_disappeared_times, time_mode):
    query_times = [float(x) for x in read_timestamps(EXP_DATA_DIR)]
    map_names = [float(n) for n in read_map_names(EXP_DATA_DIR)]
    x = (np.array(query_times) - query_times[0]) / 1e9
    metrics = list(data[list(data.keys())[0]].keys())
    gt_appeared_times = (np.array(list(gt_appeared_times)) - query_times[0]) / 1e9
    gt_disappeared_times = (np.array(list(gt_disappeared_times)) - query_times[0]) / 1e9

    # Convert data to slices.
    new_data = {}
    for method, m_data in data.items():
        # TMP
        if method != "M36_Optimistic":
            continue
        new_data[method] = {}
        for metric in metrics:
            new_data[method][metric] = get_4d_data_slice(
                m_data[metric], map_names, query_times, time_mode
            )

    # Plot.
    plot_metrics_overview(new_data, metrics, new_data.keys(), x, time_mode)
    for method in new_data.keys():
        plot_overseg(
            new_data[method],
            x,
            time_mode,
            method,
            gt_appeared_times,
            gt_disappeared_times,
        )
        plot_changes(
            new_data[method],
            x,
            time_mode,
            method,
            gt_appeared_times,
            gt_disappeared_times,
        )


def plot_changes(data, x, time_mode, method, gt_appeared_times, gt_disappeared_times):
    plt.figure(figsize=(12, 12))
    x_labels = {
        "Robot": "Experiment Time [s]",
        "Query": "Query Time [s]",
        "Online": "Online Time [s]",
    }
    x_label = x_labels[time_mode]
    x_lim = (x[0] - 1, x[-1] + 1)

    print(time_mode)

    # Appeared.
    plt.subplot(3, 2, 1)
    plt.title("Objects Appeared Raw")
    tp = data["AppearedTP"]
    fp = data["AppearedFP"]
    fn = data["AppearedFN"]
    tn = data["AppearedTN"]
    hp = data["AppearedHallucinatedP"]
    mn = data["AppearedMissedP"]
    plt.plot(x, tp, "b")
    plt.plot(x, fp, "r")
    plt.plot(x, fn, "k")
    plt.plot(x, tn, "g")
    plt.plot(x, hp, "r--")
    plt.plot(x, mn, "k--")
    plot_gt_change_lines(list(gt_appeared_times))
    plt.ylabel("Number of Objects [1]")
    plt.xlabel(x_label)
    plt.ylim(bottom=-0.9)
    plt.xlim(x_lim)
    plt.legend(["TP", "FP", "FN", "TN", "HP", "MP"])

    # Compute summary metrics.
    prec, rec, f1 = compute_metrics(tp, fp, fn)

    plt.subplot(3, 2, 2)
    plt.title("Objects Appeared Summary")
    plt.plot(x, prec * 100, "b")
    plt.plot(x, rec * 100, "g")
    plt.plot(x, f1 * 100, "k")
    fp_new = hp + fp
    fn_new = mn + fn
    prec, rec, f1 = compute_metrics(tp, fp_new, fn_new)
    plt.plot(x, prec * 100, "b--")
    plt.plot(x, rec * 100, "g--")
    plt.plot(x, f1 * 100, "k--")
    plot_gt_change_lines(list(gt_appeared_times))
    plt.ylabel("Metric Value [%]")
    plt.xlabel(x_label)
    plt.ylim(bottom=-5, top=105)
    plt.xlim(x_lim)
    plt.legend(["Precision", "Recall", "F1", "Incl. H+M"])

    # Disappeared.
    plt.subplot(3, 2, 3)
    plt.title("Objects Disappeared Raw")
    tp2 = data["DisappearedTP"]
    fp2 = data["DisappearedFP"]
    fn2 = data["DisappearedFN"]
    tn2 = data["DisappearedTN"]
    hp2 = data["DisappearedHallucinatedP"]
    mn2 = data["DisappearedMissedP"]
    plt.plot(x, tp2, "b")
    plt.plot(x, fp2, "r")
    plt.plot(x, fn2, "k")
    plt.plot(x, tn2, "g")
    plt.plot(x, hp2, "r--")
    plt.plot(x, mn2, "k--")
    plot_gt_change_lines(list(gt_disappeared_times))
    plt.ylabel("Number of Objects [1]")
    plt.xlabel(x_label)
    plt.ylim(bottom=-0.9)
    plt.xlim(x_lim)
    plt.legend(["TP", "FP", "FN", "TN", "HP", "MP"])

    # Compute summary metrics.
    prec, rec, f1 = compute_metrics(tp2, fp2, fn2)

    plt.subplot(3, 2, 4)
    plt.title("Objects Disappeared Summary")
    plt.plot(x, prec * 100, "b")
    plt.plot(x, rec * 100, "g")
    plt.plot(x, f1 * 100, "k")
    fp2_new = hp2 + fp2
    fn2_new = mn2 + fn2
    prec, rec, f1 = compute_metrics(tp2, fp2_new, fn2_new)
    plt.plot(x, prec * 100, "b--")
    plt.plot(x, rec * 100, "g--")
    plt.plot(x, f1 * 100, "k--")
    plot_gt_change_lines(list(gt_disappeared_times))
    plt.ylabel("Metric Value [%]")
    plt.xlabel(x_label)
    plt.ylim(bottom=-5, top=105)
    plt.xlim(x_lim)
    plt.legend(["Precision", "Recall", "F1", "Incl. H+M"])

    # Combine appeared and disappeared for changes overview, excluding unmatched objects.
    gt_changes_x = set()
    for t in gt_appeared_times:
        gt_changes_x.add(t)
    for t in gt_disappeared_times:
        gt_changes_x.add(t)
    gt_changes_x = list(gt_changes_x)
    plt.subplot(3, 2, 5)
    plt.title("Object Changes Raw")
    tp3 = data["AppearedTP"] + data["DisappearedTP"]
    fp3 = data["AppearedFP"] + data["DisappearedFP"]
    fn3 = data["AppearedFN"] + data["DisappearedFN"]
    tn3 = data["AppearedTN"] + data["DisappearedTN"]
    hp3 = data["AppearedHallucinatedP"] + data["DisappearedHallucinatedP"]
    mn3 = data["AppearedMissedP"] + data["DisappearedMissedP"]
    plt.plot(x, tp3, "b")
    plt.plot(x, fp3, "r")
    plt.plot(x, fn3, "k")
    plt.plot(x, tn3, "g")
    plt.plot(x, hp3, "r--")
    plt.plot(x, mn3, "k--")
    plot_gt_change_lines(gt_changes_x)
    plt.ylabel("Number of Objects [1]")
    plt.xlabel(x_label)
    plt.ylim(bottom=-0.9)
    plt.xlim(x_lim)
    plt.legend(["TP", "FP", "FN", "TN", "HP", "MP"])

    # Compute summary metrics.
    prec, rec, f1 = compute_metrics(tp3, fp3, fn3)

    plt.subplot(3, 2, 6)
    plt.title("Object Changes Summary")
    plt.plot(x, prec * 100, "b")
    plt.plot(x, rec * 100, "g")
    plt.plot(x, f1 * 100, "k")
    fp3_new = hp3 + fp3
    fn3_new = mn3 + fn3
    prec, rec, f1 = compute_metrics(tp3, fp3_new, fn3_new)
    plt.plot(x, prec * 100, "b--")
    plt.plot(x, rec * 100, "g--")
    plt.plot(x, f1 * 100, "k--")
    plot_gt_change_lines(gt_changes_x)
    plt.ylabel("Metric Value [%]")
    plt.xlabel(x_label)
    plt.ylim(bottom=-5, top=105)
    plt.xlim(x_lim)
    plt.legend(["Precision", "Recall", "F1", "Incl. H+M"])

    plt.tight_layout()

    # Save.
    file_name = os.path.join(
        OUTPUT_DEST,
        f"{method}_object_changes_{time_mode}.png",
    )
    print(f"Saving plot to '{file_name}'.")
    if not os.path.isdir(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=PLOT_DPI)
    plt.close()


def plot_overseg(data, x, time_mode, method, gt_appeared_times, gt_disappeared_times):
    plt.figure(figsize=(12, 12))

    # NOTE: Just hardcode the different plots for now.
    # Fields: NumGtLoaded	NumDsgLoaded	NumGtNotLoaded	NumDsgNotLoaded	NumOversegmented
    # NumUndersegmented	NumCorrect	MeanOversegmentationDegree	MeanUndersegmentationDegree
    # MaxOversegmentationDegree	MaxUndersegmentationDegree	AppearedTP	DisappearedTP
    # AppearedFP	DisappearedFP	AppearedFN	DisappearedFN	AppearedTN	DisappearedTN
    # NumObjDetected	NumObjMissed	NumObjHallucinated

    x_labels = {
        "Robot": "Experiment Time [s]",
        "Query": "Query Time [s]",
        "Online": "Online Time [s]",
    }
    x_label = x_labels[time_mode]
    x_lim = (x[0] - 1, x[-1] + 1)
    gt_changes_x = set()
    for t in gt_appeared_times:
        gt_changes_x.add(t)
    for t in gt_disappeared_times:
        gt_changes_x.add(t)
    gt_changes_x = list(gt_changes_x)

    plt.subplot(3, 2, 1)
    plt.title("Object Segmentation Accuracy")
    plt.plot(x, data["NumOversegmented"], "r--")
    plt.plot(x, data["NumUndersegmented"], "k:")
    plt.plot(x, data["NumCorrect"], "b")
    plot_gt_change_lines(gt_changes_x)
    plt.ylabel("Number of Objects [1]")
    plt.xlabel(x_label)
    plt.ylim(bottom=-0.9)
    plt.xlim(x_lim)
    plt.legend(["Overseg", "Underseg", "Correct"])

    plt.subplot(3, 2, 2)
    plt.title("Object Segmentation Degree")
    plt.plot(x, data["MeanOversegmentationDegree"], "r-")
    plt.plot(x, data["MeanUndersegmentationDegree"], "b-")
    plt.plot(x, data["MaxOversegmentationDegree"], "r:")
    plt.plot(x, data["MaxUndersegmentationDegree"], "b:")
    plot_gt_change_lines(gt_changes_x)
    plt.ylabel("Missegmentation Degree [-]")
    plt.xlabel(x_label)
    plt.ylim(bottom=-0.9)
    plt.xlim(x_lim)
    plt.legend(["Mean Overseg", "Mean Underseg", "Max Overseg", "Max Underseg"])

    # Combine appeared and disappeared for changes overview, excluding unmatched objects.
    plt.subplot(3, 2, 3)
    plt.title("Object Changes Raw")
    tp = data["AppearedTP"] + data["DisappearedTP"]
    fp = data["AppearedFP"] + data["DisappearedFP"]
    fn = data["AppearedFN"] + data["DisappearedFN"]
    tn = data["AppearedTN"] + data["DisappearedTN"]
    hp = data["AppearedHallucinatedP"] + data["DisappearedHallucinatedP"]
    mn = data["AppearedMissedP"] + data["DisappearedMissedP"]
    plt.plot(x, tp, "b")
    plt.plot(x, fp, "r")
    plt.plot(x, fn, "k")
    plt.plot(x, tn, "g")
    plt.plot(x, hp, "r--")
    plt.plot(x, mn, "k--")
    plot_gt_change_lines(gt_changes_x)
    plt.ylabel("Number of Objects [1]")
    plt.xlabel(x_label)
    plt.ylim(bottom=-0.9)
    plt.xlim(x_lim)
    plt.legend(["TP", "FP", "FN", "TN", "HP", "MP"])

    # Compute summary metrics.
    prec, rec, f1 = compute_metrics(tp, fp, fn)

    plt.subplot(3, 2, 4)
    plt.title("Object Changes Summary")
    plt.plot(x, prec * 100, "b")
    plt.plot(x, rec * 100, "g")
    plt.plot(x, f1 * 100, "k")
    fp += hp
    fn += mn
    prec, rec, f1 = compute_metrics(tp, fp, fn)
    plt.plot(x, prec * 100, "b--")
    plt.plot(x, rec * 100, "g--")
    plt.plot(x, f1 * 100, "k--")
    plot_gt_change_lines(gt_changes_x)
    plt.ylabel("Metric Value [%]")
    plt.xlabel(x_label)
    plt.ylim(bottom=-5, top=105)
    plt.xlim(x_lim)
    plt.legend(["Precision", "Recall", "F1", "Incl. H+M"])

    plt.subplot(3, 2, 5)
    plt.title("Object Detection")
    plot_gt_change_lines(gt_changes_x)
    plt.plot(x, data["NumObjDetected"], "b")
    plt.plot(x, data["NumObjMissed"], "r")
    plt.plot(x, data["NumObjHallucinated"], "k")
    plot_gt_change_lines(gt_changes_x)
    plt.ylabel("Number of Objects [1]")
    plt.ylim(bottom=-0.9)
    plt.xlabel(x_label)
    plt.xlim(x_lim)
    plt.legend(["Detected", "Missed", "Hallucinated"])

    prec, rec, f1 = compute_metrics(
        data["NumObjDetected"], data["NumObjHallucinated"], data["NumObjMissed"]
    )

    plt.subplot(3, 2, 6)
    plt.title("Object Detection Summary")
    plot_gt_change_lines(gt_changes_x)
    plt.plot(x, prec * 100, "b")
    plt.plot(x, rec * 100, "g")
    plt.plot(x, f1 * 100, "k")
    plot_gt_change_lines(gt_changes_x)
    plt.ylabel("Metric Value [%]")
    plt.xlabel(x_label)
    plt.ylim(bottom=-5, top=105)
    plt.xlim(x_lim)
    plt.legend(["Precision", "Recall", "F1"])

    plt.tight_layout()

    # Save.
    file_name = os.path.join(OUTPUT_DEST, f"{method}_objects_{time_mode}.png")
    print(f"Saving plot to '{file_name}'.")
    if not os.path.isdir(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=PLOT_DPI)
    plt.close()


def plot_metrics_overview(data, metrics, names, x, time_mode):
    plt.figure(figsize=(20, 16))
    n_cols = int(np.ceil(np.sqrt(len(metrics))))
    n_rows = int(np.ceil(len(metrics) / n_cols))
    for i, m in enumerate(metrics):
        plt.subplot(n_rows, n_cols, i + 1)
        for n in names:
            plt.plot(x, data[n][m])
        plt.title(metrics[i])
        if time_mode == "Robot":
            plt.xlabel("Experiment Time [s]")
        elif time_mode == "Query":
            plt.xlabel("Query Time [s]")
        else:
            plt.xlabel("Online Time [s]")
        plt.ylabel(metrics[i])
        plt.ylim(bottom=0)
        if i == 0:
            plt.legend(names)
    plt.tight_layout()

    # Save.
    file_name = os.path.join(OUTPUT_DEST, f"objects_{time_mode}_all.png")
    print(f"Saving plot to '{file_name}'.")
    if not os.path.isdir(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=PLOT_DPI)
    plt.close()


def plot_gt_change_lines(x_values):
    y_min, y_max = plt.gca().get_ylim()
    plt.vlines(x_values, ymin=0, ymax=y_max, colors="gray", alpha=0.3)


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
