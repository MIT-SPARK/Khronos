#! /usr/bin/python3

import os
import numpy as np
from matplotlib import pyplot as plt
from utils import (
    read_csv,
    read_timestamps,
    read_object_data,
    get_4d_data_slice,
    augment_object_metrics,
    read_map_names,
    read_gt_changes,
)

"""
Plot reconstruction metrics for different thresholds and different ground-truths.
"""


# Which plot to create.
PLOT_TO_USE = "office"  # apartment, office, khronos_v, pm_v, khronos_ablation


# Default Params (apartment):
class Config:
    data_dir = "/mnt/c/Users/DerFu/Documents/khronos/data"
    output_dest = "/mnt/c/Users/DerFu/Documents/khronos/plots/comp_apartment"
    gt_changes = (
        "/mnt/c/Users/DerFu/Documents/khronos/ground_truth/apartment/gt_changes.csv"
    )
    linestyles = ["b-", "r-", "g-", "b--", "r--", "g--"]
    methods = [
        "khronos/apartment_gt/results/Middle/Optimistic",
        "panoptic_mapping/apartment_gt/results/Panoptic/Mapping",
        "hydra/apartment_gt/results/Hydra/Hydra",
        "khronos/apartment_kimera/results/Middle/Optimistic",
        "panoptic_mapping/apartment_kimera/results/Panoptic/Mapping",
        "hydra/apartment_kimera/results/Hydra/Hydra",
    ]
    method_labels = [
        "Khronos (GT)",
        "Panoptic (GT)",
        "Hydra (GT)",
        "Khronos (Kimera)",
        "Panoptic (Kimera)",
        "Hydra (Kimera)",
    ]


def use_tmp():
    config = Config()
    config.methods = [
        "khronos/apartment_gt/results/Middle/Optimistic",
        "panoptic_mapping/apartment_gt/results/Panoptic/Mapping",
        "hydra/apartment_gt/results/Hydra/Hydra",
        "khronos/apartment_gt/results/Middle/Optimistic",
        "panoptic_mapping/apartment_gt/results/Panoptic/Mapping1",
        "hydra/apartment_gt/results/Hydra/Hydra_before",
    ]
    config.method_labels = [
        "Khronos (new)",
        "Panoptic (new)",
        "Hydra (new)",
        "Khronos (old)",
        "Panoptic (old)",
        "Hydra (old)",
    ]
    config.output_dest = "/mnt/c/Users/DerFu/Documents/khronos/plots/new_eval"
    return config


def use_apartment():
    # Order: Khronos, Panoptic, Hydra, First all GT, then all Kimera.
    return Config()


def use_office():
    config = Config()
    config.methods = [
        "khronos/office_gt/results/Middle/Optimistic",
        "panoptic_mapping/office_gt/results/Panoptic/Mapping",
        "hydra/office_gt/results/Hydra/Hydra",
        "khronos/office_kimera/results/Middle/Optimistic",
        "panoptic_mapping/office_kimera/results/Panoptic/Mapping",
        "hydra/office_kimera/results/Hydra/Hydra",
    ]
    config.method_labels = [
        "Khronos (GT)",
        "Panoptic (GT)",
        "Hydra (GT)",
        "Khronos (Kimera)",
        "Panoptic (Kimera)",
        "Hydra (Kimera)",
    ]
    config.output_dest = "/mnt/c/Users/DerFu/Documents/khronos/plots/comp_office"
    config.gt_changes = (
        "/mnt/c/Users/DerFu/Documents/khronos/ground_truth/office/gt_changes.csv"
    )
    return config


def use_khronos_v():
    config = Config()
    config.methods = [
        "khronos/khronos_new/results/Middle/Optimistic",
        "khronos/apartment_gt/results/Middle/Optimistic",
        "khronos/khronos_new_eval/results/Middle/Optimistic",
    ]
    config.method_labels = ["Now", "Before", "New Eval"]
    config.output_dest = "/mnt/c/Users/DerFu/Documents/khronos/plots/tmp"
    return config


def use_khronos_ablation():
    config = Config()
    config.method_labels = ["01", "02", "03", "04", "05", "06", "07", "08"]
    config.methods = [
        f"khronos/apartment_gt/results/Middle/C{m}" for m in config.method_labels
    ]
    config.output_dest = "/mnt/c/Users/DerFu/Documents/khronos/plots/tmp"
    config.linestyles = ["r", "g", "b", "c", "m", "y", "gray", "k"]
    return config


def use_pm_v():
    config = Config()
    config.methods = [
        "panoptic_mapping/apartment_gt/results/Panoptic/Mapping",
        "panoptic_mapping/apartment_gt/results/Panoptic/Mapping2",
        "panoptic_mapping/apartment_gt/results/Panoptic/Mapping3",
        "panoptic_mapping/apartment_kimera/results/Panoptic/Mapping",
        "panoptic_mapping/apartment_kimera/results/Panoptic/Mapping2",
        "panoptic_mapping/apartment_kimera/results/Panoptic/Mapping3",
    ]
    config.method_labels = [
        "GT Surface",
        "GT Centroid",
        "GT Surf-Sub",
        "Kimera Surface",
        "Kimera Centroid",
        "Kimera Surf-Sub",
    ]
    config.output_dest = "/mnt/c/Users/DerFu/Documents/khronos/plots/pm_apartment"
    return config


def get_config():
    if PLOT_TO_USE == "apartment":
        return use_apartment()
    elif PLOT_TO_USE == "office":
        return use_office()
    elif PLOT_TO_USE == "khronos_v":
        return use_khronos_v()
    elif PLOT_TO_USE == "pm_v":
        return use_pm_v()
    elif PLOT_TO_USE == "khronos_ablation":
        return use_khronos_ablation()
    elif PLOT_TO_USE == "tmp":
        return use_tmp()
    else:
        raise ValueError(f"Unknown plot to use: {PLOT_TO_USE}")


def main():
    # data[method_index][metric]=values.
    print(f"Plotting '{PLOT_TO_USE}'.")
    config = get_config()

    print(f"Loading data.")
    bg_data, object_data = parse_data(config.data_dir, config.methods)

    # Get common data
    timestamps = [
        read_timestamps(os.path.join(config.data_dir, m).split("/results/")[0])
        for m in config.methods
    ]
    map_names = [
        [
            int(x)
            for x in read_map_names(
                os.path.join(config.data_dir, m).split("/results/")[0]
            )
        ]
        for m in config.methods
    ]
    print("Augmenting object data.")
    for d in object_data:
        augment_object_metrics(d)

    # Plot
    print(f"Plotting.")
    plot_background(bg_data, timestamps, config)
    plot_object(object_data, map_names, timestamps, config)
    plot_segmentation(object_data, map_names, timestamps, config)
    print("Done.")


def plot_object(data, map_names, timestamps, config):
    metrics = [
        "ObjectPrecision",
        "ObjectRecall",
        "ObjectF1",
        "ChangePrecision",
        "ChangeRecall",
        "ChangeF1",
        "ChangePrecisionIncl",
        "ChangeRecallIncl",
        "ChangeF1Incl",
    ]
    metric_names = metrics
    conversion_factors = [100] * len(metrics)
    gt_changes = []
    if config.gt_changes is not None:
        gt_appeared_times, gt_disappeared_times = read_gt_changes(config.gt_changes)
        gt_changes = [
            (x - timestamps[0][0]) / 1e9
            for x in gt_appeared_times.union(gt_disappeared_times)
        ]

    # Plot.
    for time_mode in ["Robot", "Query", "Online"]:
        plt.figure(figsize=(12, 12))
        n_cols = int(np.ceil(np.sqrt(len(metrics))))
        n_rows = int(np.ceil(len(metrics) / n_cols))
        for i, m in enumerate(metrics):
            plt.subplot(n_rows, n_cols, i + 1)
            for j, d in enumerate(data):
                x = (timestamps[j] - timestamps[0][0]) / 1e9
                y = get_4d_data_slice(d[m], map_names[j], timestamps[j], time_mode)
                plt.plot(x[: len(y)], y * conversion_factors[i], config.linestyles[j])
            plt.title(metric_names[i])
            plt.xlabel(f"{time_mode} Time [s]")
            plt.xlim(left=-1)
            plt.ylabel(metric_names[i])
            plot_gt_change_lines(gt_changes)
            if i == 0:
                plt.legend(config.method_labels)
        plt.tight_layout()

        # Save.
        file_name = os.path.join(config.output_dest, f"Object_{time_mode}.png")
        print(f"Saving plot to '{file_name}'.")
        if not os.path.isdir(config.output_dest):
            os.makedirs(config.output_dest)
        plt.savefig(file_name, dpi=600)


def plot_segmentation(data, map_names, timestamps, config):
    metrics = [
        "NumOversegmented",
        "NumUndersegmented",
        "NumCorrect",
        "MeanOversegmentationDegree",
        "MeanUndersegmentationDegree",
        "MaxOversegmentationDegree",
        "MaxUndersegmentationDegree",
    ]
    metric_names = metrics
    conversion_factors = [1] * len(metrics)

    # Plot.
    for time_mode in ["Robot", "Query", "Online"]:
        plt.figure(figsize=(12, 12))
        n_cols = int(np.ceil(np.sqrt(len(metrics))))
        n_rows = int(np.ceil(len(metrics) / n_cols))
        for i, m in enumerate(metrics):
            plt.subplot(n_rows, n_cols, i + 1)
            for j, d in enumerate(data):
                x = (timestamps[j] - timestamps[0][0]) / 1e9
                # print(
                #     f"Plotting {method_labels[j]}, {time_mode}, {m}, {num_maps} maps."
                # )
                y = get_4d_data_slice(d[m], map_names[j], timestamps[j], time_mode)
                plt.plot(x[: len(y)], y * conversion_factors[i], config.linestyles[j])
            plt.title(metric_names[i])
            plt.xlabel(f"{time_mode} Time [s]")
            plt.xlim(left=-1)
            plt.ylabel(metric_names[i])
            if i == 0:
                plt.legend(config.method_labels)
        plt.tight_layout()

        # Save.
        file_name = os.path.join(config.output_dest, f"Segmentation_{time_mode}.png")
        print(f"Saving plot to '{file_name}'.")
        if not os.path.isdir(config.output_dest):
            os.makedirs(config.output_dest)
        plt.savefig(file_name, dpi=600)


def plot_background(data, timestamps, config):
    # Metrics to evaluate.
    metrics = [
        "Accuracy@0.5",
        "Completeness@0.5",
        "RMSE@0.5",
        "MAD@0.5",
        "Chamfer@0.5",
    ]
    metric_names = [
        "Accuracy [%]",
        "Completeness [%]",
        "RMSE [cm]",
        "MAD [cm]",
        "Chamfer [cm]",
    ]
    conversion_factors = [100, 100, 100, 100, 100]

    # Plot.
    plt.figure(figsize=(14, 10))
    n_cols = int(np.ceil(np.sqrt(len(metrics))))
    n_rows = int(np.ceil(len(metrics) / n_cols))
    for i, m in enumerate(metrics):
        plt.subplot(n_rows, n_cols, i + 1)
        for j, d in enumerate(data):
            x = (timestamps[j] - timestamps[0][0]) / 1e9
            plt.plot(x[: len(d[m])], d[m] * conversion_factors[i], config.linestyles[j])
        plt.title(metric_names[i])
        plt.xlabel("Experiment Time [s]")
        plt.xlim(left=-1)
        plt.ylabel(metric_names[i])
        if i == 0:
            plt.legend(config.method_labels)
            plt.ylim(bottom=70, top=100)
    plt.tight_layout()

    # Save.
    file_name = os.path.join(config.output_dest, "Background.png")
    print(f"Saving plot to '{file_name}'.")
    if not os.path.isdir(config.output_dest):
        os.makedirs(config.output_dest)
    plt.savefig(file_name, dpi=600)


def parse_data(data_dir, methods):
    # Add all metrics to a single data struct. method= <change>_<reconciler>.
    # data[method][metric]=values
    bg_data = []
    object_data = []

    # Get all reconcilers.
    for method in methods:
        bg_file = os.path.join(data_dir, method, "background_mesh.csv")
        if not os.path.exists(bg_file):
            print(f"File '{bg_file}' not found, skipping.")
        else:
            bg_data.append(read_csv(bg_file))

        object_file = os.path.join(data_dir, method, "static_objects.csv")
        if not os.path.exists(object_file):
            print(f"File '{object_file}' not found, skipping.")
        else:
            method_components = method.split("/results/")[1].split("/")
            data = read_object_data(data_dir + "/" + method.split("/results/")[0])
            method_name = method_components[0] + "_" + method_components[1]
            object_data.append(data[method_name])

    return bg_data, object_data


def plot_gt_change_lines(x_values):
    y_min, y_max = plt.gca().get_ylim()
    plt.vlines(x_values, ymin=0, ymax=y_max, colors="gray", alpha=0.3)


if __name__ == "__main__":
    main()
