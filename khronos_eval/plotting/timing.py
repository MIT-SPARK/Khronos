#!/usr/bin/env python

import os
from matplotlib import pyplot as plt
from utils import (
    read_timing_stats,
    read_timing_raw,
    read_change_detection_timings,
    read_reconciliation_timings,
    read_timestamps,
)

EXP_DIR = "/mnt/c/Users/DerFu/Documents/khronos/data/khronos/office_kimera"
OUTPUT_DIR = "/mnt/c/Users/DerFu/Documents/khronos/plots/timing/office"
DPI = 300

FIELDS = ["active_window/all", "frontend/all", "backend/all"]


def main():
    stats = read_timing_stats(os.path.join(EXP_DIR, "timing", "stats.csv"))
    hierarchy = create_timer_hierarchy(stats)
    augment_aw_stats(stats, hierarchy)

    # Print hierarchy.
    # print("Timer hierarchy:")
    # print_hierarchy(hierarchy)

    plot_overview(stats, hierarchy)
    table_from_stats(stats, FIELDS)
    plot_timings_raw(FIELDS)
    plot_offline_timings()


def plot_offline_timings():
    cd_data = read_change_detection_timings(EXP_DIR)
    rec_data = read_reconciliation_timings(EXP_DIR)

    # TMP: Only use the given change detector and reconciler.
    cd_data = cd_data["Middle"]
    rec_data = rec_data["Middle"]["Optimistic"]
    timestamps = read_timestamps(EXP_DIR)
    timestamps = (timestamps - timestamps[0]) * 1e-9

    # Print keys.
    print(cd_data.keys())
    print(rec_data.keys())

    # Plot.
    plt.figure()
    cd = cd_data["change_detection_recompute/all"] * 1000
    rec = rec_data["reconcile/all"] * 1000
    plt.plot(timestamps, cd)
    plt.plot(timestamps, rec)
    plt.plot(timestamps, cd + rec)
    plt.ylabel("Runtime [ms]")
    plt.xlabel("Experiment Time [s]")
    # plt.semilogy()
    plt.legend(["Change Detection", "Reconciliation", "Total"])
    plt.title("Objects + Mesh")
    plt.tight_layout()
    file_name = os.path.join(OUTPUT_DIR, "offline_with_mesh.png")
    os.makedirs(os.path.dirname(file_name), exist_ok=True)
    plt.savefig(file_name, dpi=DPI)
    print(f"Saved '{file_name}'.")
    plt.close()

    plt.figure()
    cd -= cd_data["change_detection_recompute/background"] * 1000
    rec -= rec_data["merge_mesh/all"] * 1000
    plt.plot(timestamps, cd)
    plt.plot(timestamps, rec)
    plt.plot(timestamps, cd + rec)
    plt.ylabel("Runtime [ms]")
    plt.xlabel("Experiment Time [s]")
    # plt.semilogy()
    plt.legend(["Change Detection", "Reconciliation", "Total"])
    plt.title("Objects Only")
    plt.tight_layout()
    file_name = os.path.join(OUTPUT_DIR, "offline_objects_only.png")
    os.makedirs(os.path.dirname(file_name), exist_ok=True)
    plt.savefig(file_name, dpi=DPI)
    print(f"Saved '{file_name}'.")
    plt.close()


def plot_timings_raw(fields):
    # Read data.
    data = [
        read_timing_raw(os.path.join(EXP_DIR, "timing", f + "_timing_raw.csv"))
        for f in fields
    ]
    t_min = min([d["stamps"][0] for d in data])

    # Plot.
    plt.figure()
    for d in data:
        plt.plot((d["stamps"] - t_min) * 1e-9, d["duration"] * 1000)
    plt.ylabel("Runtime [ms]")
    plt.xlabel("Experiment Time [s]")
    # plt.semilogy()
    plt.legend(fields)
    plt.tight_layout()
    file_name = os.path.join(OUTPUT_DIR, "timings_raw.png")
    os.makedirs(os.path.dirname(file_name), exist_ok=True)
    plt.savefig(file_name, dpi=DPI)
    print(f"Saved '{file_name}'.")
    plt.close()


def table_from_stats(stats, fields):
    # Print table.
    print("Timer | Mean | Std | Min | Max")
    print("--- | --- | --- | --- | ---")
    for name in fields:
        mean = stats[name]["mean"] * 1000
        std = stats[name]["std"] * 1000
        min = stats[name]["min"] * 1000
        max = stats[name]["max"] * 1000
        print(f"{name} | {mean:.2f} | {std:.2f} | {min:.2f} | {max:.2f}")


def plot_overview(stats, hierarchy):
    # Plot main categories.
    plt.rcParams.update({"font.size": 12})
    categories = [x for x in hierarchy if hierarchy[x]]
    plt.figure(figsize=(len(categories) * 2.5 + 3, 9))
    axes = []
    legends = []
    num_plots = len(categories) + 1
    for i in range(num_plots):
        plt.subplot(1, num_plots, i + 1)
        axes.append(plt.gca())
        # Expand all items
        items = []
        max = None
        cat = ""
        if i == 0:
            plt.title("frame")
            items = ["active_window", "evaluate", "visualize"]
            max = "frame"
            legends.append(items)
            items = [item + "/all" for item in items]
        else:
            cat = categories[i - 1]
            for item in hierarchy[cat]:
                if item == "all":
                    max = item
                else:
                    items.append(item)
            plt.title(cat)

        # Plot bar
        current = 0
        for item in items:
            val = stats[(cat + "/" + item) if cat else item]["mean"] * 1000
            plt.bar(0, val, bottom=current)
            current += val

        # Fill with to total.
        if max is not None:
            max_val = stats[(cat + "/" + max) if cat else max]["mean"] * 1000
            if max_val > current:
                plt.bar(0, max_val - current, bottom=current, color="lightgray")
                items.append("other")
            else:
                height = current * 0.005
                plt.bar(0, height, bottom=(max_val - height / 2), width=1, color="k")
                items.append("frame")

        # Format plot.
        plt.xticks([])
        if i == 0:
            plt.ylabel("Mean runtime [ms]")
        else:
            legends.append(items)

    plt.tight_layout()
    for ax, legend in zip(axes, legends):
        box = ax.get_position()
        ax.set_position(
            [box.x0, box.y0 + box.height * 0.4, box.width, box.height * 0.6]
        )
        ax.legend(
            legend,
            loc="upper center",
            bbox_to_anchor=(0.5, -0.0),
            fancybox=True,
            shadow=False,
            ncol=1,
        )

    file_name = os.path.join(OUTPUT_DIR, "overview.png")
    os.makedirs(os.path.dirname(file_name), exist_ok=True)
    plt.savefig(file_name, dpi=DPI)
    print(f"Saved '{file_name}'.")
    plt.close()


def create_timer_hierarchy(stats):
    # Create a hierarchy of all timer names.
    hierarchy = {}
    for name in stats:
        parts = name.split("/")
        node = hierarchy
        for part in parts:
            if part not in node:
                node[part] = {}
            node = node[part]
    return hierarchy


def print_hierarchy(hierarchy, indent=0):
    for name in hierarchy:
        print(" " * indent + name)
        print_hierarchy(hierarchy[name], indent + 2)


def augment_aw_stats(stats, hierarchy):
    # Port hidden stats to the main categories.
    for name in ["motion_detection", "object_detection", "tracking"]:
        stats["active_window/" + name] = stats[name + "/all"]
        hierarchy["active_window"][name] = {}


if __name__ == "__main__":
    main()
