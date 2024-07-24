#! /usr/bin/python3

import os
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, StrMethodFormatter
from mpl_toolkits.mplot3d import Axes3D
from utils import read_csv, read_timestamps
from scipy import interpolate

"""
Plot reconstruction metrics for different thresholds and different ground-truths.
"""

# Params:
# Experiment data directory
# EXP_DATA_DIR = "/mnt/c/Users/DerFu/Documents/khronos/data/previous/mesh_test_data"
EXP_DATA_DIR = "/mnt/c/Users/DerFu/Documents/khronos/data/apartment_gt"


# Plotting
ROBOT_COLOR = "b"
QUERY_COLOR = "g"
REF_COLOR = "k"
REF_COLOR2 = "gray"
THIN_LINE = 1
THICK_LINE = 2
FIG_SIZE = 8

# Auto-generated params:
OUTPUT_DEST = EXP_DATA_DIR + "/plots"


def main():
    print(f"Plotting 4D stuff for '{EXP_DATA_DIR}'.")
    # data[header] = values
    # data = read_csv(EXP_DATA_DIR + "/metrics/mesh_10000.csv")
    data = read_csv(EXP_DATA_DIR + "/results/Middle/Optimistic/static_objects.csv")

    # Metrics to evaluate.
    metric = "NumObjDetected"
    # metric = "MeshMemory"
    # "Accuracy@0.1",
    # "Completeness@0.1",
    # "RMSE@0.1",
    # "MAD@0.1",
    # "Chamfer@0.1",

    metric_name = "Correct Objects Detected"
    # metric_name = "Memory [MB]"
    # "Accuracy [%]",
    # "Completeness [%]",
    # "RMSE [cm]",
    # "MAD [cm]",
    # "Chamfer [cm]",

    # Get timestamps
    x = read_timestamps(EXP_DATA_DIR)
    x = (x - x[0]) / 1e9

    # Get data into grid. data[robot_time_idx][query_time_idx] = value
    y = []
    for i, _ in enumerate(x):
        y.append([])
        for _ in range(i + 1):
            y[-1].append(data[metric][i] * 100)

    # Plot.
    output_file = os.path.join(OUTPUT_DEST, f"{metric_name}_4d")
    plot_4d(x, y, metric_name, output_file_name=output_file)


def plot_4d(
    x,
    data,
    metric_name,
    plot_name=None,
    output_file_name=None,
    view_azimuth=-50,
    view_elevation=40,
    plot_surface=False,
    surface_resolution=None,
    query_detail_indices=[],
    plot_hidden_axes=True,
    save_intermediate_frames=False,
    fig=None,
    ax=None,
):
    """
    x: 1D array of x values (time steps)
    data: Queryable data[robot_time_idx][query_time_idx] = value
    metric_name: Name of the metric to plot.
    Kwargs:
        plot_name: Title of the plot. None to ignore.
        output_file_name: File to save the plot to. None to not save.
        view_azimuth: Azimuth angle of the plot.
        view_elevation: Elevation angle of the plot.
        plot_surface: Whether to plot the surf. Requires nvidia graphics to work properly.
        surface_resolution: Resolution to plot draw the surf. None to use the original resolution.
        query_detail_indices: Indices of x values to plot the query details for.
        plot_hidden_axes: Whether to plot the hidden axes.
    """

    # Setup.
    fig, ax = plt.subplots(
        subplot_kw={"projection": "3d"}, figsize=(FIG_SIZE, FIG_SIZE)
    )
    ax.view_init(view_elevation, view_azimuth)
    frame_counter = 0
    formatter = StrMethodFormatter("{x:,.1f}")
    ax.zaxis.set_major_locator(LinearLocator(10))
    ax.zaxis.set_major_formatter(formatter)
    ax.set_zlabel(metric_name, labelpad=15)
    plt.ylabel("Robot Time [s]")
    plt.xlabel("Query Time [s]")
    plt.xlim([x[0], x[-1]])
    plt.ylim([x[0], x[-1]])
    if plot_name is not None:
        plt.title(plot_name)
    x_idx = [i for i, _ in enumerate(x)]

    # Find the z-limit.
    z_max = -np.inf
    for i in x_idx:
        for j in range(i):
            z_max = max(z_max, data[i][j])
    ax.set_zlim(0, z_max)
    plt.tight_layout()
    if save_intermediate_frames and output_file_name is not None:
        plt.savefig(f"{output_file_name}_{frame_counter}.png", dpi=600)
        frame_counter += 1

    # Optional: Plot surf (rendering fails without nvidia graphics)
    if plot_surface:
        plot_surf(x, data, fig, ax, surface_resolution)

    # Plot the real-time axis
    ax.plot3D(
        x,
        x,
        [data[i][i] for i in x_idx],
        linewidth=THICK_LINE,
        color=ROBOT_COLOR,
        linestyle="-",
    )
    ax.plot3D(
        x,
        x,
        [0 for _ in x_idx],
        "k",
        linewidth=THIN_LINE,
        color=ROBOT_COLOR,
        linestyle="-",
    )
    ax.plot3D(
        [x[-1], x[-1]],
        [x[-1], x[-1]],
        [0, data[-1][-1]],
        linewidth=THIN_LINE,
        color=ROBOT_COLOR,
        linestyle="-",
    )
    ax.plot3D(
        [x[0], x[0]],
        [x[0], x[0]],
        [0, data[0][0]],
        linewidth=THIN_LINE,
        color=ROBOT_COLOR,
        linestyle="-",
    )

    if save_intermediate_frames and output_file_name is not None:
        plt.savefig(f"{output_file_name}_{frame_counter}.png", dpi=600)
        frame_counter += 1

    # Plot the query-time marginal.
    ax.plot3D(
        [0 for _ in x_idx],
        x,
        [data[i][0] for i in x_idx],
        linewidth=THICK_LINE,
        color=ROBOT_COLOR,
        linestyle="-",
    )
    if plot_hidden_axes:
        ax.plot3D(
            [x[0], x[0]],
            [x[-1], x[-1]],
            [0, data[-1][0]],
            linewidth=THIN_LINE,
            color=ROBOT_COLOR,
            linestyle="--",
        )
        ax.plot3D(
            [x[0], x[0]],
            [x[0], x[-1]],
            [0, 0],
            linewidth=THIN_LINE,
            color=ROBOT_COLOR,
            linestyle="--",
        )

    if save_intermediate_frames and output_file_name is not None:
        plt.savefig(f"{output_file_name}_{frame_counter}.png", dpi=600)
        frame_counter += 1

    # Plot the robot-time marginal.
    ax.plot3D(
        x,
        [x[-1] for _ in x_idx],
        [data[-1][i] for i in x_idx],
        linewidth=THICK_LINE,
        color=ROBOT_COLOR,
        linestyle="-",
    )
    if plot_hidden_axes:
        ax.plot3D(
            [x[0], x[-1]],
            [x[-1], x[-1]],
            [0, 0],
            linewidth=THIN_LINE,
            color=ROBOT_COLOR,
            linestyle="--",
        )
    if save_intermediate_frames and output_file_name is not None:
        plt.savefig(f"{output_file_name}_{frame_counter}.png", dpi=600)
        frame_counter += 1

    # Plot specific query highlights.
    for x_query in query_detail_indices:
        plot_detail_query(x, data, ax, x_query)
        if save_intermediate_frames and output_file_name is not None:
            plt.savefig(f"{output_file_name}_{frame_counter}.png", dpi=600)
            frame_counter += 1

    # Save.
    if output_file_name is not None:
        print(f"Saving plot to '{output_file_name}'.png.")
        if not os.path.exists(os.path.dirname(OUTPUT_DEST)):
            os.makedirs(os.path.dirname(OUTPUT_DEST))
        plt.savefig(output_file_name + ".png", dpi=600)
    print("Done.")


def plot_detail_query(x, data, ax, query_idx):
    """
    x: 1D array of x values (time steps)
    data: Queryable data[robot_time_idx][query_time_idx] = value
    x_query: x_value to plot query at.
    """
    # Plot reference line in black.
    ax.plot3D(
        [x[query_idx], x[query_idx]],
        [x[0], x[query_idx]],
        [0, 0],
        linewidth=THIN_LINE,
        color=REF_COLOR,
        linestyle="--",
    )
    ax.plot3D(
        [x[query_idx], x[query_idx]],
        [x[query_idx], x[query_idx]],
        [0, data[query_idx][query_idx]],
        linewidth=THIN_LINE,
        color=REF_COLOR,
        linestyle="--",
    )
    ax.plot3D(
        [x[query_idx], x[query_idx]],
        [x[query_idx], x[-1]],
        [data[query_idx][query_idx], data[query_idx][query_idx]],
        linewidth=THIN_LINE,
        color=REF_COLOR,
        linestyle="--",
    )
    ax.plot3D(
        [x[query_idx], x[query_idx]],
        [x[-1], x[query_idx]],
        [0, 0],
        linewidth=THIN_LINE,
        color=REF_COLOR2,
        linestyle="--",
    )
    ax.plot3D(
        [x[query_idx], x[query_idx]],
        [x[-1], x[-1]],
        [data[query_idx][query_idx], 0],
        linewidth=THIN_LINE,
        color=REF_COLOR2,
        linestyle="--",
    )

    # Plot the information improvement in green.
    ax.plot3D(
        [x[query_idx] for _ in x[query_idx:]],
        x[query_idx:],
        [data[i + query_idx][query_idx] for i, _ in enumerate(x[query_idx:])],
        linewidth=THICK_LINE,
        color=QUERY_COLOR,
        linestyle="-",
    )
    ax.plot3D(
        [x[query_idx], x[query_idx]],
        [x[-1], x[-1]],
        [data[query_idx][query_idx], data[-1][query_idx]],
        linewidth=THIN_LINE,
        color=QUERY_COLOR,
        linestyle="-",
    )


def plot_surf(x, data, fig, ax, resolution=None):
    xx, yy = np.meshgrid(x, x)
    z = np.zeros_like(xx)
    for i, _ in enumerate(x):
        for j in range(i + 1):
            z[i, j] = data[i][j]

    # xx, yy = np.meshgrid(t, t)
    if resolution is not None:
        interp = interpolate.RegularGridInterpolator((x, x), z)
        x_new = np.linspace(x[0], x[-1], resolution)
        xx, yy = np.meshgrid(x_new, x_new)
        zz = interp((xx, yy))
    else:
        zz = z
    surf = ax.plot_surface(yy, xx, zz, cmap=cm.coolwarm, linewidth=0, antialiased=True)

    fig.colorbar(surf, shrink=0.5, aspect=5)


if __name__ == "__main__":
    main()
