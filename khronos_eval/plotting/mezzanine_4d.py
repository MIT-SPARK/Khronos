#! /usr/bin/python3

import os
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, StrMethodFormatter
from mpl_toolkits.mplot3d import Axes3D


# Params:
OUTPUT_DEST = "/mnt/c/Users/DerFu/Documents/khronos/plots/4dmaps/mezzanine"

QUERY_INDICES = [0.25, 0.5, 0.75]

# Plotting
ROBOT_COLOR = "b"
QUERY_COLOR = "g"
REF_COLOR = "k"
REF_COLOR2 = "gray"
THIN_LINE = 1
THICK_LINE = 2
MAJOR_LINE = 3
FIG_SIZE = 8


PLOT_NUM = 6  # 0 - 6, -1 for all


def main():
    # Define the data to plot.
    resolution = 200
    robot_slice_time = 125
    query_slice_time = 25
    objects_appeared = np.array(
        [0, 0, 0, 0, 11.5, 16, 127, 138, 139, 195]
    )  # Timestamps of when objects appeared
    objects_disappeared = [
        [69, 31],
        [204, 197],
    ]  # Timestamps of when objects disappeared <observed, belief>
    objects_new = [[82, 37]]  # Timestamps of when objects appeared <observed, belief>

    # Create the data
    x = np.linspace(0, 200, resolution)  # 226.6
    y = []
    for robot_idx, robot_time in enumerate(x):
        y.append([])
        for query_idx in range(robot_idx + 1):
            # Compute the number of objects present at this time
            y[-1].append(
                num_present(
                    robot_time,
                    x[query_idx],
                    objects_appeared,
                    objects_disappeared,
                    objects_new,
                )
            )

    # Plot
    os.makedirs(OUTPUT_DEST, exist_ok=True)
    if PLOT_NUM >= 0:
        plot_4d(
            x,
            y,
            "Number of Objects Present",
            output_file_name=OUTPUT_DEST + "/num_obj",
            query_detail_indices=[round(i * len(x)) for i in QUERY_INDICES],
            save_intermediate_frames=False,
            r_time=robot_slice_time,
            q_time=query_slice_time,
            plot_num=PLOT_NUM,
        )
    else:
        for i in range(7):
            plot_4d(
                x,
                y,
                "Number of Objects Present",
                output_file_name=OUTPUT_DEST + "/num_obj",
                query_detail_indices=[round(i * len(x)) for i in QUERY_INDICES],
                save_intermediate_frames=False,
                r_time=robot_slice_time,
                q_time=query_slice_time,
                plot_num=i,
            )


def num_present(robot, query, objects_appeared, objects_disappeared, objects_new):
    num_obj = np.sum(objects_appeared <= robot)
    for obj in objects_disappeared:
        if obj[0] <= robot and obj[1] <= query:
            num_obj -= 1
    for obj in objects_new:
        if obj[0] <= robot and obj[1] <= query:
            num_obj += 1
    return num_obj


def plot_4d(
    x,
    data,
    metric_name,
    plot_name=None,
    output_file_name=None,
    view_azimuth=-50,
    view_elevation=40,
    query_detail_indices=[],
    plot_hidden_axes=True,
    save_intermediate_frames=False,
    ax=None,
    r_time=0,
    q_time=0,
    plot_num=0,
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
    formatter = StrMethodFormatter("{x:.0f}")
    ax.zaxis.set_major_locator(LinearLocator(11))
    ax.zaxis.set_major_formatter(formatter)
    ax.set_zlabel(metric_name, labelpad=15)
    plt.ylabel("Robot Time [s]")
    plt.xlabel("Query Time [s]")
    plt.xlim([x[0], x[-1]])
    plt.ylim([x[0], x[-1]])
    if plot_name is not None:
        plt.title(plot_name)
    x_idx = [i for i, _ in enumerate(x)]
    r_idx = len([xx for xx in x if xx <= r_time])
    q_idx = len([xx for xx in x if xx <= q_time])

    # Find the z-limit.
    z_max = -np.inf
    for i in x_idx:
        for j in range(i):
            z_max = max(z_max, data[i][j])
    ax.set_zlim(0, z_max)
    plt.tight_layout()

    # plt.savefig(f"{output_file_name}_0.png", dpi=600)

    # Comptue times
    x1 = [xx for xx in x if xx <= r_time]
    x2 = [xx for xx in x if xx >= q_time and xx <= r_time]
    x4 = [xx for xx in x if xx < q_time]
    x2_offset = len(x4)
    x3 = [xx for xx in x if xx >= r_time]
    x5 = [xx for xx in x if xx < r_time and xx >= q_time]
    x5_offset = len([xx for xx in x if xx < q_time])
    x3_offset = len(x5) + x5_offset

    # Plot the full 4D plot.
    if plot_num > 5:
        # Plot the query-time marginal.
        ax.plot3D(
            [0 for _ in x_idx],
            x,
            [data[i][0] for i in x_idx],
            linewidth=THICK_LINE,
            color=ROBOT_COLOR,
            linestyle="-",
        )
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
        # Plot the robot-time marginal.
        ax.plot3D(
            x,
            [x[-1] for _ in x_idx],
            [data[-1][i] for i in x_idx],
            linewidth=THICK_LINE,
            color=ROBOT_COLOR,
            linestyle="-",
        )
        ax.plot3D(
            [x[0], x[-1]],
            [x[-1], x[-1]],
            [0, 0],
            linewidth=THIN_LINE,
            color=ROBOT_COLOR,
            linestyle="--",
        )
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
        # Plot the remainder of the detail insets.
        ax.plot3D(
            [q_time for _ in x5],
            x5,
            [data[i + x5_offset][q_idx] for i, _ in enumerate(x5)],
            linewidth=THIN_LINE,
            color=QUERY_COLOR,
            linestyle="-",
        )
        ax.plot3D(
            x4,
            [r_time for _ in x4],
            [data[r_idx][i] for i, _ in enumerate(x4)],
            linewidth=THIN_LINE,
            color=QUERY_COLOR,
            linestyle="-",
        )

    # Plot the robot time intersection
    if plot_num > 3:
        ax.plot3D(
            [q_time, q_time],
            [0, r_time],
            [0, 0],
            linewidth=THIN_LINE,
            color=REF_COLOR2,
            linestyle="--",
        )
        ax.plot3D(
            [q_time, q_time],
            [r_time, x[-1]],
            [0, 0],
            linewidth=THIN_LINE,
            color=REF_COLOR2 if plot_num == 4 else REF_COLOR,
            linestyle="--",
        )
    if plot_num > 4:
        ax.plot3D(
            [q_time for _ in x3],
            x3,
            [data[i + x3_offset][q_idx] for i, _ in enumerate(x3)],
            linewidth=MAJOR_LINE,
            color=QUERY_COLOR,
            linestyle="-",
        )
        ax.plot3D(
            [q_time, q_time],
            [x[-1], x[-1]],
            [0, data[-1][q_idx]],
            linewidth=THIN_LINE,
            color=QUERY_COLOR,
            linestyle="-",
        )

    # Plot the query time intersection
    if plot_num > 1:
        ax.plot3D(
            [0, q_time],
            [r_time, r_time],
            [0, 0],
            linewidth=THIN_LINE,
            color=REF_COLOR2,
            linestyle="--",
        )
        ax.plot3D(
            [q_time, r_time],
            [r_time, r_time],
            [0, 0],
            linewidth=THIN_LINE,
            color=REF_COLOR2 if plot_num == 2 else REF_COLOR,
            linestyle="--",
        )
        ax.plot3D(
            [r_time, x[-1]],
            [r_time, r_time],
            [0, 0],
            linewidth=THIN_LINE,
            color=REF_COLOR2,
            linestyle="--",
        )
    if plot_num > 2:
        ax.plot3D(
            x2,
            [r_time for _ in x2],
            [data[r_idx][i + x2_offset] for i, _ in enumerate(x2)],
            linewidth=MAJOR_LINE,
            color=QUERY_COLOR,
            linestyle="-",
        )
        ax.plot3D(
            [q_time, q_time],
            [r_time, r_time],
            [0, data[r_idx][q_idx]],
            linewidth=THIN_LINE,
            color=QUERY_COLOR,
            linestyle="-",
        )

    # Plot the real-time axis.
    if plot_num > 0:
        ax.plot3D(
            x1,
            x1,
            [data[i][i] for i, _ in enumerate(x1)],
            linewidth=MAJOR_LINE,
            color=ROBOT_COLOR,
            linestyle="-",
        )
        ax.plot3D(
            x1,
            x1,
            [0 for _ in x1],
            linewidth=THIN_LINE,
            color=ROBOT_COLOR,
            linestyle="-",
        )
        ax.plot3D(
            [x1[-1], x1[-1]],
            [x1[-1], x1[-1]],
            [0, data[len(x1) - 1][len(x1) - 1]],
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

    # Save.
    plt.savefig(f"{output_file_name}_{plot_num}.png", dpi=600)
    print(f"Saving plot to '{output_file_name}_{plot_num}.png'.")
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


if __name__ == "__main__":
    main()
