#! /usr/bin/python3

import os
import numpy as np
from matplotlib import pyplot as plt
from utils import read_csv, read_timestamps, read_map_names
from plot_4d import *

"""
Plot numbers of objects and their presence state of a spatio-temporal map.
NOTE(lschmid): To run this, first run 'evaluate_pipeline' on the target dir.
"""

# Params:
# Experiment data directory
EXP_DATA_DIR = "/mnt/c/Users/DerFu/Documents/khronos/data/apartment_gt"
OUTPUT_DEST = "/mnt/c/Users/DerFu/Documents/khronos/plots/4dmaps/apartment_gt"
CHANGE_DETECTOR_NAME = "Middle"
RECONCILER_NAME = "Optimistic"  # Conservative, Optimistic

# Plotting params:
X_RESOLUTION = 1000

# Auto-generated params:
DATA_DIR = os.path.join(EXP_DATA_DIR, "pipeline", CHANGE_DETECTOR_NAME, RECONCILER_NAME)


def main():
    print(f"Reading spatio-temporal map data for '{DATA_DIR}'.")
    # data[robot_timestamp][object_id][property]=values. Properties: "first_present", "last_present"
    data, robot_times = read_data()
    # x = np.linspace(0, robot_times[-1], X_RESOLUTION)

    # Metrics to plot. Metrics are functions metric(data, x_query)->y_query.
    # metrics = [num_objects_total, num_objects_present, num_objects_absent]
    # metric_names = ["Total Objects", "Present Objects", "Absent Objects"]
    metrics = [num_objects_present]
    metric_names = ["Present Objects"]

    # Plot the final spatio-temporal-map queried at different times.
    # plot_final(data, x, metrics, metric_names)

    # Plot overview.
    # plot_overview(data, x, metrics, metric_names, robot_times)

    # Plot time progression.
    # plot_time_progression(data, x, metrics, metric_names, robot_times, num_times=5)

    # Plot 4D plots.
    plot_4ds(data, metrics, metric_names, robot_times)


def plot_4ds(data, metrics, metric_names, robot_times):
    for i, m in enumerate(metrics):
        # Get data into grid.
        y = []
        for j, _ in enumerate(robot_times):
            y.append([])
            for k in range(j + 1):
                y[-1].append(m(data[j], robot_times[k]))

        # Plot.
        plot_surface = False
        output_file = os.path.join(
            OUTPUT_DEST, f"{metric_names[i]}{'_surf' if plot_surface else ''}"
        )
        os.makedirs(OUTPUT_DEST, exist_ok=True)
        plot_4d(
            robot_times,
            y,
            metric_names[i],
            output_file_name=output_file,
            query_detail_indices=[9, 19],
            plot_surface=plot_surface,
            surface_resolution=500,
            save_intermediate_frames=True,
        )


def plot_time_progression(data, x, metrics, metric_names, robot_times, num_times=5):
    print("Plotting spatio-temporal-map time progression.")
    fig, ax = plt.subplots(1, len(metrics), figsize=(12, 4.5))
    plt.rcParams.update({"font.size": 13})
    indices = [
        int(np.round((i + 1) * (len(robot_times) - 1) / num_times))
        for i in range(num_times)
    ]
    for i, metric in enumerate(metrics):
        for index in indices:
            local_data = data[index]
            y = np.array([metric(local_data, x_i) for x_i in x])
            ax[i].plot(x, y)
        ax[i].set_title(metric_names[i])
        ax[i].set_xlabel("Query Time [s]")
        ax[i].set_ylabel(f"Number of {metric_names[i]} [1]")
    plt.tight_layout()

    fig.subplots_adjust(bottom=0.25)
    ax[1].legend(
        labels=[f"t_robot={robot_times[index]:.2f}s" for index in indices],
        loc="upper center",
        bbox_to_anchor=(0.5, -0.2),
        fancybox=False,
        shadow=False,
        ncol=num_times,
    )

    # Save.
    file_name = os.path.join(OUTPUT_DEST, "progression.png")
    print(f"Saving plot to '{file_name}'.")
    if not os.path.exists(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=600)
    print("Done.")


def plot_overview(data, x, metrics, metric_names, robot_times):
    print("Plotting spatio-temporal-map overview.")
    plt.figure(figsize=(15, 15))
    plt.rcParams.update({"font.size": 13})
    n_cols = int(np.ceil(np.sqrt(len(robot_times))))
    n_rows = int(np.ceil(len(robot_times) / n_cols))
    for i, robot_time in enumerate(robot_times):
        plt.subplot(n_rows, n_cols, i + 1)
        local_data = data[i]
        for m in metrics:
            y = np.array([m(local_data, x_i) for x_i in x])
            plt.plot(x, y)
        plt.axvline(robot_time, c="gray", alpha=0.7)
        plt.title(f"Robot Time: {robot_time:.2f}s")
        plt.xlabel("Query Time [s]")
        plt.ylabel("Number of Objects [1]")
        plt.ylim(0, 70)  # TMP for this dataset.
        if i == 0:
            plt.legend(metric_names + ["Robot Time"])
    plt.tight_layout()

    # Save.
    file_name = os.path.join(OUTPUT_DEST, "overview.png")
    print(f"Saving plot to '{file_name}'.")
    if not os.path.exists(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=600)
    print("Done.")


def plot_final(data, x, metrics, metric_names):
    print("Plotting single spatio-temporal-map.")
    plt.figure(figsize=(8, 8))
    local_data = data[-1]
    for m in metrics:
        y = np.array([m(local_data, x_i) for x_i in x])
        plt.plot(x, y)
    plt.xlabel("Query Time [s]")
    plt.title("Final Spatio-Temporal Map")
    plt.ylabel("Number of Objects [1]")
    plt.legend(metric_names)
    plt.tight_layout()

    # Save.
    file_name = os.path.join(OUTPUT_DEST, "final_object_presence.png")
    print(f"Saving plot to '{file_name}'.")
    if not os.path.exists(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=600)
    print("Done.")


def read_data():
    map_names = read_map_names(EXP_DATA_DIR)
    time_stamps = read_timestamps(EXP_DATA_DIR)
    t_start = time_stamps[0]
    data = []
    time_stamps = [(t - t_start) / 1e9 for t in time_stamps]
    for map_name in map_names:
        new_data = read_csv(
            os.path.join(DATA_DIR, f"{map_name}_object_presence_times.csv"), by_row=True
        )
        # Normalize presense times.
        for d in new_data.values():
            d["first_present"] = (d["first_present"] - t_start) / 1e9
            d["last_present"] = (d["last_present"] - t_start) / 1e9
        data.append(new_data)
    return data, time_stamps


# Metrics.
def num_objects_total(data, x):
    """
    Total number of objects in the map.
    """
    return len([d for d in data.values() if d["first_present"] <= x])


def num_objects_present(data, x):
    """
    Number of objects present at time x.
    """
    return len(
        [d for d in data.values() if d["first_present"] <= x and d["last_present"] >= x]
    )


def num_objects_absent(data, x):
    """
    Number of objects absent at time x.
    """
    return len(
        [d for d in data.values() if d["first_present"] <= x and d["last_present"] < x]
    )


if __name__ == "__main__":
    main()
