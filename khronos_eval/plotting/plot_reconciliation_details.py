#! /usr/bin/python3

import os
import numpy as np
from matplotlib import pyplot as plt
from utils import (
    read_timestamps,
    TimeConverter,
    read_map_names,
    read_associations,
    read_object_presence_times,
    read_csv_by_row,
    object_string_to_id,
)
import sys
import csv

# Params:
SCENE = "office"  # apartment, office
SUFFIX = "kimera_new"  # gt, kimera
EXP_DIR = f"/mnt/c/Users/DerFu/Documents/khronos/data/khronos/{SCENE}_{SUFFIX}"
OUTPUT_DEST = f"/mnt/c/Users/DerFu/Documents/khronos/plots/reconciliation_details/{SCENE}_{SUFFIX}"
CHANGE_DETECTOR = "AB"  # Middle,
RECONCILER = "Optimistic"  # "Optimistic"
GT_CHANGES_FILE = (
    f"/mnt/c/Users/DerFu/Documents/khronos/ground_truth/{SCENE}/gt_changes.csv"
)
PLOT_DPI = 300

PLOT_CHANGES_IN_RECONCILIATION = True
MAP_ID = 43 if SCENE == "office" else 34  # 34, 43

# Change detectors
CD_RES = 1 * 1e9  # nanoseconds
CD_WINDOW = 5
# CD_THRESH = [0.5, 0.6, 0.7]
CD_THRESH = [50, 100, 500]
CD_ABSOLUTE = True

SEARCH = False


def main():
    # Increase CSV field size limit for the cd details lol.
    csv.field_size_limit(sys.maxsize)

    if SEARCH:
        search()
        return

    # Presence data[object_id][field] = values (vector)
    # fields = "observed", "estimated", "first_seen", "last_seen", "change_details"
    print(f"Loading data.")
    data = read_object_presence_times(
        os.path.join(
            EXP_DIR,
            "pipeline",
            CHANGE_DETECTOR,
            RECONCILER,
            f"{MAP_ID:05d}_object_presence_times.csv",
        )
    )

    # associations[map_id][timestamp][from_id] = to_id or None
    gt_to_dsg, dsg_to_gt = read_associations(
        os.path.join(
            EXP_DIR,
            "eval_visualization",
            CHANGE_DETECTOR,
            RECONCILER,
            "associations.csv",
        )
    )
    times = TimeConverter(read_timestamps(EXP_DIR))
    map_names = [int(x) for x in read_map_names(EXP_DIR)]
    gt_changes = parse_gt_changes(GT_CHANGES_FILE)

    # Plot
    plot_presence(
        data,
        gt_to_dsg[MAP_ID][times.timestamps[MAP_ID]],
        dsg_to_gt[MAP_ID][times.timestamps[MAP_ID]],
        times,
        gt_changes,
    )

    # Plot change details for all objects.
    changed_ids = (
        [37, 40, 301, 11, 290, 149, 151, 221]
        if SCENE == "office"
        else [6, 9, 138, 8, 130, 25]
    )
    # plot_change_density(
    #     data,
    #     changed_ids,
    #     times,
    # )

    # plot_cd_all(
    #     data,
    #     changed_ids,
    #     times,
    # )

    # plot_cd_emulation(
    #     data,
    #     gt_to_dsg[MAP_ID][times.timestamps[MAP_ID]],
    #     dsg_to_gt[MAP_ID][times.timestamps[MAP_ID]],
    #     times,
    #     gt_changes,
    # )


def search():
    # specify search parameters
    reconcilers = ["Optimistic"]
    change_detectors = ["Middle", "First", "A", "B"]
    window_sizes = [5, 10, 15]
    decision_rules = {
        False: [0.5, 0.6, 0.7, 0.8],
        True: [30, 50, 100, 300, 500],
    }  # True: absolute, False: relative, thresholds

    # search
    times = TimeConverter(read_timestamps(EXP_DIR))
    changed_ids = (
        [37, 40, 301, 11, 290, 149, 151, 221]
        if SCENE == "office"
        else [6, 9, 138, 8, 130, 25]
    )
    results = []
    for det in change_detectors:
        for rec in reconcilers:
            data = read_object_presence_times(
                os.path.join(
                    EXP_DIR,
                    "pipeline",
                    det,
                    rec,
                    f"{MAP_ID:05d}_object_presence_times.csv",
                )
            )
            for window in window_sizes:
                global CD_WINDOW
                CD_WINDOW = window
                for abs, thresholds in decision_rules.items():
                    global CD_ABSOLUTE
                    global CD_THRESH
                    CD_ABSOLUTE = abs
                    CD_THRESH = thresholds
                    result = plot_cd_all(
                        data,
                        changed_ids,
                        times,
                        save_name=f"{det}_{rec}_W{window}_{'Abs' if abs else 'Rel'}.png",
                    )
                    results.append(result)

    # print results
    idx = 0
    print("Reconciler,Detector,Window,Absolute,Threshold,TP,FP,FN,Pre,Rec,F1")
    for det in change_detectors:
        for recon in reconcilers:
            for window in window_sizes:
                for abs, thresholds in decision_rules.items():
                    for thresh in thresholds:
                        vals = results[idx][thresh]
                        tp = vals["tp"]
                        fp = vals["fp"]
                        fn = vals["fn"]
                        pre = tp / (tp + fp) * 100 if tp + fp > 0 else np.nan
                        rec = tp / (tp + fn) * 100 if tp + fn > 0 else np.nan
                        f1 = 2 * pre * rec / (pre + rec)
                        print(
                            f"{det},{recon},{window},{'Abs' if abs else 'Rel'},{thresh},{tp},{fp},{fn},{pre},{rec},{f1}"
                        )
                    idx += 1


# Datastructure to store change detection result for a pre-merge object.
class ObjectState:
    def __init__(self, first_seen, last_seen):
        self.first_seen = first_seen
        self.last_seen = last_seen
        self.presence_before = None
        self.absence_before = None
        self.presence_after = None
        self.absence_after = None
        self.first_present = None
        self.last_present = None


def cd_decision_rule(x, y_pres_w, y_abs_w, threshold, forward=True):
    # Current decision rule: find first time index where confidence > threshold, with absence preceding presence.
    # Returns present_time, absent_time. Values are None if not found.
    if CD_ABSOLUTE:
        c_present = y_pres_w > threshold
        c_absent = y_abs_w > threshold
    else:
        confidence = y_pres_w / (y_pres_w + y_abs_w)
        c_present = confidence > threshold
        c_absent = confidence < 1 - threshold
    absent_time = None
    present_time = None
    if np.sum(c_present) > 0:
        present_time = np.max(x[c_present]) if forward else np.min(x[c_present])
    if np.sum(c_absent) > 0:
        absent_time = np.min(x[c_absent]) if forward else np.max(x[c_absent])
    if present_time is not None and absent_time is not None:
        if (forward and absent_time < present_time) or (
            not forward and absent_time > present_time
        ):
            present_time = None
    return present_time, absent_time


def plot_cd_emulation(data, gt_to_dsg, dsg_to_gt, times, gt_changes):
    # Plot.
    plt.figure(figsize=(6, 10))
    num_plots = len(gt_changes)
    for i, gt_id in enumerate(gt_changes):
        plt.subplot(num_plots, 1, i + 1)
        plt.title(f"GT O({gt_id})")
        plt.xlim([0, times.max_s])

        # Find all DSG objects that are associated with this GT object.
        dsg_ids = []
        for from_id, to_id in dsg_to_gt.items():
            if to_id == gt_id:
                dsg_ids.append(from_id)

        chosen_dsg_id = -1
        if gt_id in gt_to_dsg:
            chosen_dsg_id = gt_to_dsg[gt_id]
        ylabels = ["GT Changes"]
        y_ticks = [0]
        y_curr = 0
        for dsg_id in dsg_ids:
            ylabels.append(f"DSG O({dsg_id}){'*' if dsg_id == chosen_dsg_id else ''}")
            y_curr += 1
            y_ticks.append(y_curr)

            # Plot final presence.
            object_data = emulate_cd_and_rec(data[dsg_id])
            for obj in object_data:
                # Plot estimated and observed presence.
                if obj.first_present is not None and obj.last_present is not None:
                    plt.plot(
                        [times.toSec(obj.first_present), times.toSec(obj.last_present)],
                        [y_curr] * 2,
                        color="g",
                        linewidth=1,
                    )

                plt.plot(
                    [times.toSec(obj.first_seen), times.toSec(obj.last_seen)],
                    [y_curr] * 2,
                    color="b",
                    linewidth=2,
                )

                # Plot change details.
                if obj.presence_before is not None:
                    plt.scatter(
                        times.toSec(obj.presence_before),
                        y_curr,
                        color="b",
                        marker="o",
                    )
                if obj.absence_before is not None:
                    plt.scatter(
                        times.toSec(obj.absence_before),
                        y_curr,
                        color="r",
                        marker="x",
                    )
                if obj.presence_after is not None:
                    plt.scatter(
                        times.toSec(obj.presence_after),
                        y_curr,
                        color="b",
                        marker="o",
                    )
                if obj.absence_after is not None:
                    plt.scatter(
                        times.toSec(obj.absence_after),
                        y_curr,
                        color="r",
                        marker="x",
                    )
                y_curr += 0.2
        plt.ylim([-0.2, y_curr])
        plt.yticks(y_ticks, ylabels)

        # Plot true presence.
        appeared = sorted(times.toSecs(gt_changes[gt_id]["appeared"]))
        disappeared = sorted(times.toSecs(gt_changes[gt_id]["disappeared"]))
        plt.scatter(
            appeared,
            np.zeros_like(appeared),
            color="g",
            marker="*",
        )
        plt.scatter(
            disappeared,
            np.zeros_like(disappeared),
            color="r",
            marker="x",
        )
        while appeared or disappeared:
            t_start = 0
            t_end = times.times[-1]
            if appeared and not disappeared:
                t_start = appeared[0]
                appeared.clear()
            elif disappeared and not appeared:
                t_end = disappeared[0]
                disappeared.clear()
            else:
                if appeared[0] < disappeared[0]:
                    t_start = appeared[0]
                    appeared.pop(0)
                    t_end = disappeared[0]
                    disappeared.pop(0)
                else:
                    t_end = disappeared[0]
                    disappeared.pop(0)
            plt.plot(
                [t_start, t_end],
                [0, 0],
                color="gray",
                linewidth=2,
            )

    # Format and legend.
    plt.xlabel("Experiment Time [s]")
    plt.tight_layout()
    ax = plt.gca()
    (line1,) = ax.plot([-1, -1], [0, 1], color="k", label="Final Presence")
    (line2,) = ax.plot([-1, -1], [0, 1], color="b", label="Observed")
    (line3,) = ax.plot([-1, -1], [0, 1], color="g", label="Estimated")
    box = ax.get_position()
    plt.gca().set_position(
        [box.x0, box.y0 + box.height * 0.2, box.width, box.height * 0.8]
    )
    plt.gca().legend(
        loc="upper center",
        bbox_to_anchor=(0.4, -0.35),
        fancybox=True,
        shadow=False,
        ncol=3,
        handles=[line1, line2, line3],
    )

    # Save.
    file_name = os.path.join(OUTPUT_DEST, f"emulated.png")
    print(f"Saving plot to '{file_name}'.")
    if not os.path.isdir(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=PLOT_DPI)
    plt.close()


def emulate_cd_and_rec(datum):
    # Returns a list of ObjectState objects.
    result = []
    observed = datum["observed"]
    for i in range(int(len(observed) / 2)):
        state = ObjectState(observed[2 * i], observed[2 * i + 1])
        result.append(state)
    return result


def plot_cd_all(data, changed_ids, times, save_name=None):
    # Plot.
    # Presence data[object_id][field] = values (vector)

    # Pre-filter data to re-observed objects.
    new_data = {}
    changed_data = {}
    for id, datum in data.items():
        has_data = False
        for key in [
            "cd_before_present",
            "cd_before_absent",
            "cd_after_present",
            "cd_after_absent",
        ]:
            for entry in parse_cd_details(datum[key]):
                if len(entry) > 0:
                    has_data = True
                    break
            if has_data:
                break
        if has_data:
            if id in changed_ids:
                changed_data[id] = datum
            else:
                new_data[id] = datum

    changed_data.update(new_data)
    n_rows = int(np.ceil(np.sqrt(len(changed_data))))
    n_cols = int(np.ceil(len(changed_data) / n_rows))

    plt.figure(figsize=(20, 20))
    results = {conf: {"tp": 0, "fp": 0, "fn": 0} for conf in CD_THRESH}
    for i, datum in enumerate(changed_data.values()):
        id = list(changed_data.keys())[i]
        plt.subplot(n_rows, n_cols, i + 1)
        plt.title(f"O({id}){ ' CHANGED' if id in changed_ids else ''}")
        plt.xlim([0, times.max_s])
        # NOTE(lschmid): There's several entries for each merged object. For now only plot the first one.
        # Plot the historgram of the change details.
        time_series_before, time_series_after = computetime_series(datum, CD_RES)
        x_1, y_pres_1_w, y_abs_1_w = time_series_to_linear(
            time_series_before, CD_WINDOW, False
        )
        x_2, y_pres_2_w, y_abs_2_w = time_series_to_linear(time_series_after, CD_WINDOW)
        y_pres_w = np.concatenate([y_pres_1_w, y_pres_2_w])
        y_abs_w = np.concatenate([y_abs_1_w, y_abs_2_w])
        if CD_ABSOLUTE:
            c_present = y_pres_w
            c_absent = y_abs_w
        else:
            confidence = y_pres_w / (y_pres_w + y_abs_w)
            c_present = confidence
            c_absent = 1 - confidence
            c_present[c_present < 0.5] = 0
            c_absent[c_absent < 0.5] = 0
        t = times.toSecs(np.array(np.concatenate([x_1, x_2])) * CD_RES)

        plt.bar(
            t,
            c_present,
            color="b",
            width=CD_RES / 1e9,
            alpha=0.3,
        )
        plt.bar(
            t,
            c_absent,
            color="r",
            width=CD_RES / 1e9,
            alpha=0.3,
        )
        for conf in CD_THRESH:
            plt.plot(
                [0, times.max_s],
                [conf] * 2,
                color="gray",
                linestyle="-",
                alpha=0.5,
                linewidth=1,
            )
            pres, abs = cd_decision_rule(x_1, y_pres_1_w, y_abs_1_w, conf, False)
            if pres is not None:
                plt.scatter(
                    times.toSec(pres * CD_RES), conf, color="b", marker="o", s=5
                )
            if abs is not None:
                plt.scatter(times.toSec(abs * CD_RES), conf, color="r", marker="x", s=5)
            pres2, abs2 = cd_decision_rule(x_2, y_pres_2_w, y_abs_2_w, conf, True)
            if pres2 is not None:
                plt.scatter(
                    times.toSec(pres2 * CD_RES), conf, color="b", marker="o", s=5
                )
            if abs2 is not None:
                plt.scatter(
                    times.toSec(abs2 * CD_RES), conf, color="r", marker="x", s=5
                )

            # Log results.
            has_absence = abs is not None or abs2 is not None
            if id in changed_ids:
                if has_absence:
                    results[conf]["tp"] += 1
                else:
                    results[conf]["fn"] += 1
            elif has_absence:
                results[conf]["fp"] += 1

    # Save.
    plt.tight_layout()
    file_name = os.path.join(
        OUTPUT_DEST, f"cd_all.png" if save_name is None else save_name
    )
    print(f"Saving plot to '{file_name}'.")
    if not os.path.isdir(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=PLOT_DPI)
    plt.close()
    return results


def plot_change_density(data, ids, times):
    # Plot.
    # Presence data[object_id][field] = values (vector)
    plt.figure(figsize=(25, 15))
    for i, id in enumerate(ids):
        plt.subplot(len(ids), 3, i * 3 + 1)
        plt.ylabel(f"O({id})")
        plt.xlim([0, times.max_s])
        if i == 0:
            plt.title("Rays")

        # NOTE(lschmid): There's several entries for each merged object. For now only plot the first one.
        # Plot the historgram of the change details.
        time_series_before, time_series_after = computetime_series(data[id], CD_RES)

        # Plot the time series.
        x1, y_pres_1, y_abs_1 = time_series_to_linear(time_series_before)
        x2, y_pres_2, y_abs_2 = time_series_to_linear(time_series_after)
        x = np.concatenate([x1, x2])
        y_pres = np.concatenate([y_pres_1, y_pres_2])
        y_abs = np.concatenate([y_abs_1, y_abs_2])
        t = times.toSecs(np.array(x) * CD_RES)

        plt.bar(
            t,
            y_pres,
            color="b",
            width=CD_RES / 1e9,
            alpha=0.5,
        )
        plt.bar(
            t,
            y_abs,
            color="r",
            width=CD_RES / 1e9,
            alpha=0.5,
        )
        plt.scatter(t, y_pres, color="b", s=5)
        plt.scatter(t, y_abs, color="r", s=5)

        plt.subplot(len(ids), 3, i * 3 + 2)
        if i == 0:
            plt.title("Windowed")
        plt.xlim([0, times.max_s])
        x_1, y_pres_1_w, y_abs_1_w = time_series_to_linear(
            time_series_before, CD_WINDOW, False
        )
        x_2, y_pres_2_w, y_abs_2_w = time_series_to_linear(time_series_after, CD_WINDOW)
        y_pres_w = np.concatenate([y_pres_1_w, y_pres_2_w])
        y_abs_w = np.concatenate([y_abs_1_w, y_abs_2_w])
        plt.bar(
            t,
            y_pres_w,
            color="b",
            width=CD_RES / 1e9,
            alpha=0.5,
        )
        plt.bar(
            t,
            y_abs_w,
            color="r",
            width=CD_RES / 1e9,
            alpha=0.5,
        )
        plt.scatter(t, y_pres_w, color="b", s=5)
        plt.scatter(t, y_abs_w, color="r", s=5)
        plt.subplot(len(ids), 3, i * 3 + 3)
        if i == 0:
            plt.title("Confidence")
        plt.xlim([0, times.max_s])
        confidence = y_pres_w / (y_pres_w + y_abs_w)
        c_present = confidence
        c_absent = 1 - confidence
        c_present[c_present < 0.5] = 0
        c_absent[c_absent < 0.5] = 0
        plt.bar(
            t,
            c_present,
            color="b",
            width=CD_RES / 1e9,
            alpha=0.5,
        )
        plt.bar(
            t,
            c_absent,
            color="r",
            width=CD_RES / 1e9,
            alpha=0.5,
        )
        for conf in [0.5, 0.6, 0.7]:
            plt.plot(
                [0, times.max_s],
                [conf] * 2,
                color="gray",
                linestyle="-",
                alpha=0.5,
                linewidth=1,
            )
            pres, abs = cd_decision_rule(x_1, y_pres_1_w, y_abs_1_w, conf, False)
            if pres is not None:
                plt.scatter(times.toSec(pres * CD_RES), conf, color="b", marker="o")
            if abs is not None:
                plt.scatter(times.toSec(abs * CD_RES), conf, color="r", marker="x")
            pres, abs = cd_decision_rule(x_2, y_pres_2_w, y_abs_2_w, conf, True)
            if pres is not None:
                plt.scatter(times.toSec(pres * CD_RES), conf, color="b", marker="o")
            if abs is not None:
                plt.scatter(times.toSec(abs * CD_RES), conf, color="r", marker="x")

    # Save.
    plt.tight_layout()
    file_name = os.path.join(OUTPUT_DEST, f"cd_details.png")
    print(f"Saving plot to '{file_name}'.")
    if not os.path.isdir(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=PLOT_DPI)
    plt.close()


def plot_presence(presence_data, gt_to_dsg, dsg_to_gt, times, gt_changes):
    # Plot.
    plt.figure(figsize=(6, 10))
    num_plots = len(gt_changes)
    for i, gt_id in enumerate(gt_changes):
        plt.subplot(num_plots, 1, i + 1)
        plt.title(f"GT O({gt_id})")
        plt.xlim([0, times.max_s])

        # Find all DSG objects that are associated with this GT object.
        dsg_ids = []
        for from_id, to_id in dsg_to_gt.items():
            if to_id == gt_id:
                dsg_ids.append(from_id)

        chosen_dsg_id = -1
        if gt_id in gt_to_dsg:
            chosen_dsg_id = gt_to_dsg[gt_id]
        ylabels = ["GT Changes"]
        y_ticks = [0]
        y_curr = 0
        for dsg_id in dsg_ids:
            ylabels.append(f"DSG O({dsg_id}){'*' if dsg_id == chosen_dsg_id else ''}")
            y_curr += 1
            y_ticks.append(y_curr)
            before_present = parse_cd_details(
                presence_data[dsg_id]["cd_before_present"]
            )
            before_absent = parse_cd_details(presence_data[dsg_id]["cd_before_absent"])
            after_present = parse_cd_details(presence_data[dsg_id]["cd_after_present"])
            after_absent = parse_cd_details(presence_data[dsg_id]["cd_after_absent"])

            # Plot final presence.
            first = presence_data[dsg_id]["first_seen"]
            last = presence_data[dsg_id]["last_seen"]
            for k, f in enumerate(first):
                plt.plot(
                    [times.toSec(f), times.toSec(last[k])],
                    [y_curr] * 2,
                    color="k",
                    linewidth=1.5,
                )

            # Plot estimated and observed presence.
            estimated = presence_data[dsg_id]["estimated"]
            observed = presence_data[dsg_id]["observed"]
            for k in range(int(len(observed) / 2)):
                if len(estimated) >= 2 * k + 2:
                    y_curr += 0.2
                    plt.plot(
                        [
                            times.toSec(estimated[2 * k]),
                            times.toSec(estimated[2 * k + 1]),
                        ],
                        [y_curr] * 2,
                        color="g",
                        linewidth=1,
                    )
                plt.plot(
                    [times.toSec(observed[2 * k]), times.toSec(observed[2 * k + 1])],
                    [y_curr] * 2,
                    color="b",
                    linewidth=2,
                )

                # Plot change details.
                if len(before_present) > k and PLOT_CHANGES_IN_RECONCILIATION:
                    presence = [times.toSec(x) for x in before_present[k]]
                    absence = [times.toSec(x) for x in before_absent[k]]
                    plt.scatter(
                        presence, [y_curr + 0.05] * len(presence), color="b", marker="x"
                    )
                    plt.scatter(
                        absence, [y_curr + 0.05] * len(absence), color="r", marker="x"
                    )
                    presence = [times.toSec(x) for x in after_present[k]]
                    absence = [times.toSec(x) for x in after_absent[k]]
                    plt.scatter(
                        presence, [y_curr + 0.05] * len(presence), color="b", marker="*"
                    )
                    plt.scatter(
                        absence, [y_curr + 0.05] * len(absence), color="r", marker="*"
                    )

        plt.ylim([-0.2, y_curr + 0.2])
        plt.yticks(y_ticks, ylabels)

        # Plot true presence.
        appeared = sorted(times.toSecs(gt_changes[gt_id]["appeared"]))
        disappeared = sorted(times.toSecs(gt_changes[gt_id]["disappeared"]))
        plt.scatter(
            appeared,
            np.zeros_like(appeared),
            color="g",
            marker="*",
        )
        plt.scatter(
            disappeared,
            np.zeros_like(disappeared),
            color="r",
            marker="x",
        )
        while appeared or disappeared:
            t_start = 0
            t_end = times.times[-1]
            if appeared and not disappeared:
                t_start = appeared[0]
                appeared.clear()
            elif disappeared and not appeared:
                t_end = disappeared[0]
                disappeared.clear()
            else:
                if appeared[0] < disappeared[0]:
                    t_start = appeared[0]
                    appeared.pop(0)
                    t_end = disappeared[0]
                    disappeared.pop(0)
                else:
                    t_end = disappeared[0]
                    disappeared.pop(0)
            plt.plot(
                [t_start, t_end],
                [0, 0],
                color="gray",
                linewidth=2,
            )

    # Format and legend.
    plt.xlabel("Experiment Time [s]")
    plt.tight_layout()
    ax = plt.gca()
    (line1,) = ax.plot([-1, -1], [0, 1], color="k", label="Final Presence")
    (line2,) = ax.plot([-1, -1], [0, 1], color="b", label="Observed")
    (line3,) = ax.plot([-1, -1], [0, 1], color="g", label="Estimated")
    box = ax.get_position()
    plt.gca().set_position(
        [box.x0, box.y0 + box.height * 0.2, box.width, box.height * 0.8]
    )
    plt.gca().legend(
        loc="upper center",
        bbox_to_anchor=(0.4, -0.35),
        fancybox=True,
        shadow=False,
        ncol=3,
        handles=[line1, line2, line3],
    )

    # Save.
    file_name = os.path.join(OUTPUT_DEST, f"rec_details.png")
    print(f"Saving plot to '{file_name}'.")
    if not os.path.isdir(OUTPUT_DEST):
        os.makedirs(OUTPUT_DEST)
    plt.savefig(file_name, dpi=PLOT_DPI)
    plt.close()


def computetime_series(data, resolution):
    # returns time_series_before, time_series_after. time_series[time_index] = [presence, absence]
    before_present = parse_cd_details(data["cd_before_present"])
    before_absent = parse_cd_details(data["cd_before_absent"])
    after_present = parse_cd_details(data["cd_after_present"])
    after_absent = parse_cd_details(data["cd_after_absent"])
    time_series_before = {}
    time_series_after = {}
    for val in before_present[0]:
        time_index = int(val / resolution)
        if time_index not in time_series_before:
            time_series_before[time_index] = [0, 0]
        time_series_before[time_index][0] += 1
    for val in before_absent[0]:
        time_index = int(val / resolution)
        if time_index not in time_series_before:
            time_series_before[time_index] = [0, 0]
        time_series_before[time_index][1] += 1
    for val in after_present[0]:
        time_index = int(val / resolution)
        if time_index not in time_series_after:
            time_series_after[time_index] = [0, 0]
        time_series_after[time_index][0] += 1
    for val in after_absent[0]:
        time_index = int(val / resolution)
        if time_index not in time_series_after:
            time_series_after[time_index] = [0, 0]
        time_series_after[time_index][1] += 1
    return time_series_before, time_series_after


def time_series_to_linear(time_series, window_size=1, forward=True):
    # Convert time series to linear arrays. Optionally aggregate over a window.
    x = []
    y_pres = []
    y_abs = []
    for index, values in time_series.items():
        x.append(index)
        y_pres.append(values[0])
        y_abs.append(values[1])
        if window_size > 1:
            for j in range(1, window_size):
                new_index = index + j if forward else index - j
                if new_index in time_series:
                    y_pres[-1] += time_series[new_index][0]
                    y_abs[-1] += time_series[new_index][1]
    return np.array(x), np.array(y_pres), np.array(y_abs)


def parse_gt_changes(file_name):
    # gt_changes[object_id][appeared/disappeared] = values
    data = read_csv_by_row(GT_CHANGES_FILE)
    gt_changes = {}
    for d in data:
        object_id = object_string_to_id(d["ObjectSymbol"])
        if object_id not in gt_changes:
            gt_changes[object_id] = {"appeared": [], "disappeared": []}
        appeared = np.uint64(d["AppearedAt"])
        disappeared = np.uint64(d["DisappearedAt"])
        if appeared != 0:
            gt_changes[object_id]["appeared"].append(appeared)
        if disappeared != 0:
            gt_changes[object_id]["disappeared"].append(disappeared)
    return gt_changes


def parse_cd_details(raw_vector):
    # data[idx] = timestamps
    # Uses entry '0' as object separator.
    data = []
    curr = []
    for x in raw_vector:
        if x == 0:
            data.append(np.array(curr))
            curr = []
        else:
            curr.append(x)
    data.append(np.array(curr))
    return data


if __name__ == "__main__":
    main()
