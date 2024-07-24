#! /usr/bin/python3

import numpy as np
import csv
import yaml
import os

"""
Tools to interpret experiment directories and read data.
"""


def read_csv(filename, by_row=False):
    """
    Read a CSV file and return a dictionary of numpy arrays if the values are float-castable or list of strings otherwise.
    ByColumn: assumes each column starts with a header and is followed by values. Interface: data[header] = np.array(values) or [values].
    ByRow: assumes each row starts with a name followed by a value for each header, where the first row is the headers. Interface: data[name][header] = value.
    """
    with open(filename, mode="r") as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=",")
        data = {}
        if by_row:
            # Parse all rows.
            headers = None
            for row in csv_reader:
                if headers is None:
                    headers = row
                else:
                    name = row[0]
                    data[name] = {}
                    for i, header in enumerate(headers[1:]):
                        data[name][header] = convert_csv_cell_value(row[i + 1])
        else:
            # Parse all columns (default)
            headers = None
            for row in csv_reader:
                if headers is None:
                    headers = row
                    for header in headers:
                        data[header] = []
                    continue
                for i, header in enumerate(headers):
                    data[header].append(convert_csv_cell_value(row[i]))
            for header in headers:
                if type(data[header][0]) is not str:
                    data[header] = np.array(data[header])
        return data


def read_csv_by_row(filename):
    """
    TMP: Return a list of dicts for each row.
    """
    with open(filename, mode="r") as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=",")
        data = []
        # Parse all rows.
        headers = None
        for row in csv_reader:
            if headers is None:
                headers = row
            else:
                d = {}
                for i, header in enumerate(headers):
                    if len(row) <= i:
                        d[header] = np.nan
                        continue
                    d[header] = convert_csv_cell_value(row[i])

                data.append(d)
        return data


def convert_csv_cell_value(value):
    """
    Converts a csv cell value to the appropriate type of uint64, float, or string.
    value: str input.
    """
    if value == "#NAME?":
        return np.nan
    if value.isnumeric():
        return np.uint64(value)
    try:
        return float(value)
    except ValueError:
        return value


class TimeConverter:
    """
    Helper class to convert timestamps to experiment time.
    """

    def __init__(self, timestamps):
        """
        timestamps: array of timestamps in ns.
        """
        # Bounds for scaling in ns.
        self.timestamps = timestamps
        self.min_ns = timestamps[0]
        self.max_ns = timestamps[-1]
        # Already rescaled timestamps in seconds.
        self.times = (timestamps - self.min_ns) * 1e-9
        self.max_s = self.times[-1]

    def toSec(self, timestamp):
        """
        Convert a timestamp to seconds.
        """
        if timestamp < self.min_ns:
            return 0
        elif timestamp > self.max_ns:
            return self.max_s
        return (timestamp - self.min_ns) * 1e-9

    def toStamp(self, second):
        """
        Convert a timestamp to seconds.
        """
        if second < 0:
            return self.min_ns
        elif second > self.max_s:
            return self.max_ns
        return second * 1e9 + self.min_ns

    def toSecs(self, timestamps):
        """
        Convert an array of timestamps to seconds.
        """
        return np.array([self.toSec(t) for t in timestamps])

    def toStamps(self, seconds):
        """
        Convert an array of timestamps to seconds.
        """
        return np.array([self.toStamp(s) for s in seconds])


def read_timestamps(experiment_directory):
    """
    Read all timestamps for each map from the experiment directory.
    """
    timestamps = []
    maps_dir = os.path.join(experiment_directory, "maps")
    for map_name in read_map_names(experiment_directory):
        stamp_file = os.path.join(maps_dir, map_name, "timestamp.txt")
        if not os.path.exists(stamp_file):
            print(f"Warning: No 'timestamp.txt' found for map '{map_name}'.")
            timestamps.append(np.nan)
            continue
        with open(stamp_file, "r") as f:
            timestamps.append(int(f.read()))
    return np.array(timestamps)


def read_map_names(experiment_directory):
    """
    Get all map names from the experiment directory.
    """
    maps_dir = os.path.join(experiment_directory, "maps")
    if not os.path.exists(maps_dir):
        print(f"Warning: Maps directory '{maps_dir}' does not exist.")
        return []
    map_names = [
        d
        for d in os.listdir(maps_dir)
        if os.path.isdir(os.path.join(maps_dir, d)) and d != "." and d != ".."
    ]
    map_names.sort()
    return map_names


def detect_methods(experiment_dir, results_dir_name="results"):
    """
    Get a list of all <change_detector, reconciler> tuples run in the directory.
    """
    result = []
    results_dir = os.path.join(experiment_dir, results_dir_name)
    if not os.path.exists(results_dir):
        print(f"Warning: Results directory '{results_dir}' does not exist.")
        return result
    for change in os.listdir(results_dir):
        if not os.path.isdir(os.path.join(results_dir, change)):
            continue
        if change == "." or change == "..":
            continue
        change_dir = os.path.join(results_dir, change)
        for rec in os.listdir(change_dir):
            if not os.path.isdir(os.path.join(change_dir, rec)):
                continue
            if rec == "." or rec == "..":
                continue
            result.append((change, rec))
    return result


def readYamlConfig(config_file_path):
    """
    Read a yaml config file if it exists.
    """
    if not config_file_path:
        return None

    if not os.path.exists(config_file_path):
        print(f"Warning: config file '{config_file_path}' does not exist.")
        return None
    with open(config_file_path, "r") as stream:
        try:
            config = yaml.safe_load(stream)
            return config
        except yaml.YAMLError as exc:
            print(f"Warning: Failed to read config file '{config_file_path}': {exc}")
            return None


def read_object_data(exp_data_dir, file_name="static_objects.csv"):
    """
    Read all object data from the experiment directory.
    """
    # Add all metrics to a single data struct. method= <change>_<reconciler>.
    # data[method][metric][map_name][query_time]=values
    data = {}

    # Get all reconcilers.
    for change, rec in detect_methods(exp_data_dir):
        method = change + "_" + rec
        # Read object data.
        data_file = os.path.join(exp_data_dir, "results", change, rec, file_name)
        if not os.path.exists(data_file):
            print(f"Warning: No '{file_name}' found for method '{method}', skipping.")
            continue
        csv_data = read_csv_by_row(data_file)
        # Manualy parse to rows for now.
        if not csv_data:
            print(f"The data loaded from '{data_file}' is empty, skipping.")
            continue
        data[method] = {}
        for key in csv_data[0].keys():
            if key == "Name" or key == "Query":
                continue
            data[method][key] = {}
        for row in csv_data:
            for key, value in row.items():
                if key == "Name" or key == "Query":
                    continue
                name = row["Name"]
                if name not in data[method][key]:
                    data[method][key][name] = {}
                data[method][key][row["Name"]][row["Query"]] = value
    return data


def read_dynamic_object_data(exp_data_dir):
    raw_data = read_object_data(exp_data_dir, "dynamic_objects.csv")
    result = {}
    for method, data in raw_data.items():
        result[method] = {}
        # Relable the data to have unique names.
        result[method]["DynamicNumGtLoaded"] = data["NumGtLoaded"]
        result[method]["DynamicNumDsgLoaded"] = data["NumDsgLoaded"]
        result[method]["DynamicNumGtNotLoaded"] = data["NumGtNotLoaded"]
        result[method]["DynamicNumDsgNotLoaded"] = data["NumDsgNotLoaded"]
        result[method]["DynamicTP"] = data["NumObjDetected"]
        result[method]["DynamicFN"] = data["NumObjMissed"]
        result[method]["DynamicFP"] = data["NumObjHallucinated"]
    return result


def get_4d_data_slice(data, map_names, query_times, time_mode):
    """
    Get a slice from 4D data.
    data: data[map_name][query_time]=values
    map_names: list of map names (data robot time keys)
    query_times: list of query times (data query time keys)
    time_mode: "Robot", "Query", or "Online"
    return the slice as a numpy array.
    """
    result = np.zeros((len(map_names)))
    if time_mode == "Robot":
        for i, map_name in enumerate(map_names):
            query_time = query_times[0]
            result[i] = data[map_name][query_time]
    elif time_mode == "Query":
        map_name = map_names[-1]
        for i, query_time in enumerate(query_times):
            result[i] = data[map_name][query_times[i]]
    elif time_mode == "Online":
        for i, map_name in enumerate(map_names):
            query_time = query_times[i]
            result[i] = data[map_name][query_time]
    return result


def read_gt_changes(file_name, is_config_file=False):
    """
    Reads the gt_changes.csv file from an evaluation config and returns a set of appeared and disappeared times.
    """
    gt_appeared_times = set()
    gt_disappeared_times = set()
    if not file_name:
        return gt_appeared_times, gt_disappeared_times

    if is_config_file:
        # Figure out which changes.csv to load first.

        config = readYamlConfig(file_name)
        if not config:
            print(f"Failed to read config file '{file_name}'.")
            return gt_appeared_times, gt_disappeared_times
        file_name = config["gt_changes_file"]

    changes = read_csv_by_row(file_name)
    for x in changes:
        if x["AppearedAt"] != 0:
            gt_appeared_times.add(x["AppearedAt"])
        if x["DisappearedAt"] != 0:
            gt_disappeared_times.add(x["DisappearedAt"])
    print(
        f"Read {len(gt_appeared_times) + len(gt_disappeared_times)} appeared times from gt_changes.csv."
    )
    return gt_appeared_times, gt_disappeared_times


def parse_table_data(data_dir, methods):
    # Add all metrics to a single data struct. method= <change>_<reconciler>.
    # data[method_index][metric][map_no][timestamp]=value.
    data = [{} for _ in methods]

    # Get all reconcilers.
    for i, method in enumerate(methods):
        method_components = method.split("/results/")[1].split("/")
        method_name = method_components[0] + "_" + method_components[1]
        exp_dir = data_dir + "/" + method.split("/results/")[0]

        bg_file = os.path.join(data_dir, method, "background_mesh.csv")
        if not os.path.exists(bg_file):
            print(f"File '{bg_file}' not found, skipping.")
        else:
            # Format bg data to match object data.
            values = read_csv(bg_file)
            new_values = {}
            for metric in values:
                if metric == "Name":
                    continue
                new_values[metric] = {}
                for map in values["Name"]:
                    value = values[metric][int(map)]
                    new_values[metric][int(map)] = {
                        j: value for j in range(int(map) + 1)
                    }
            data[i].update(new_values)

        # Object data
        object_file = os.path.join(data_dir, method, "static_objects.csv")
        if not os.path.exists(object_file):
            print(f"File '{object_file}' not found, skipping.")
        else:
            obj_data = read_object_data(exp_dir)
            data[i].update(obj_data[method_name])

        # Dynamic data
        dynamic_file = os.path.join(data_dir, method, "dynamic_objects.csv")
        if not os.path.exists(dynamic_file):
            print(f"File '{dynamic_file}' not found, skipping.")
        else:
            dyn_data = read_dynamic_object_data(exp_dir)
            data[i].update(dyn_data[method_name])

    return data


def augment_object_metrics(data):
    """
    Comptue all derived metrics such as precision, recall, and f1 score.
    data[metric][map_name][query_time]=values
    """
    new_metrics = [
        "ObjectPrecision",
        "ObjectRecall",
        "ObjectF1",
        "ObjectPrecisionIncl",
        "ObjectRecallIncl",
        "ObjectF1Incl",
        "ChangePrecision",
        "ChangeRecall",
        "ChangeF1",
        "ChangePrecisionIncl",
        "ChangeRecallIncl",
        "ChangeF1Incl",
        "AppearedPrecision",
        "AppearedRecall",
        "AppearedF1",
        "AppearedPrecisionIncl",
        "AppearedRecallIncl",
        "AppearedF1Incl",
        "DisappearedPrecision",
        "DisappearedRecall",
        "DisappearedF1",
        "DisappearedPrecisionIncl",
        "DisappearedRecallIncl",
        "DisappearedF1Incl",
        "ChangeTP",
        "ChangeFP",
        "ChangeFN",
        "ChangeTN",
    ]

    header = data["NumGtLoaded"]
    for metric in new_metrics:
        data[metric] = {}
        for map_name in header.keys():
            data[metric][map_name] = {}

    for map_name, query_times in header.items():
        for query_time in query_times.keys():
            tp = data["AppearedTP"][map_name][query_time]
            fp = data["AppearedFP"][map_name][query_time]
            fn = data["AppearedFN"][map_name][query_time]
            hp = data["AppearedHallucinatedP"][map_name][query_time]
            mn = data["AppearedMissedP"][map_name][query_time]
            prec, rec, f1 = compute_metrics_single(tp, fp, fn)
            data["AppearedPrecision"][map_name][query_time] = prec
            data["AppearedRecall"][map_name][query_time] = rec
            data["AppearedF1"][map_name][query_time] = f1
            prec, rec, f1 = compute_metrics_single(tp, fp + hp, fn + mn)
            data["AppearedPrecisionIncl"][map_name][query_time] = prec
            data["AppearedRecallIncl"][map_name][query_time] = rec
            data["AppearedF1Incl"][map_name][query_time] = f1

            tp2 = data["DisappearedTP"][map_name][query_time]
            fp2 = data["DisappearedFP"][map_name][query_time]
            fn2 = data["DisappearedFN"][map_name][query_time]
            hp2 = data["DisappearedHallucinatedP"][map_name][query_time]
            mn2 = data["DisappearedMissedP"][map_name][query_time]
            prec, rec, f1 = compute_metrics_single(tp2, fp2, fn2)
            data["DisappearedPrecision"][map_name][query_time] = prec
            data["DisappearedRecall"][map_name][query_time] = rec
            data["DisappearedF1"][map_name][query_time] = f1
            prec, rec, f1 = compute_metrics_single(tp2, fp2 + hp2, fn2 + mn2)
            data["DisappearedPrecisionIncl"][map_name][query_time] = prec
            data["DisappearedRecallIncl"][map_name][query_time] = rec
            data["DisappearedF1Incl"][map_name][query_time] = f1

            prec, rec, f1 = compute_metrics_single(tp + tp2, fp + fp2, fn + fn2)
            data["ChangeTP"][map_name][query_time] = tp + tp2
            data["ChangeFP"][map_name][query_time] = fp + fp2
            data["ChangeFN"][map_name][query_time] = fn + fn2
            data["ChangeTN"][map_name][query_time] = (
                data["AppearedTN"][map_name][query_time]
                + data["DisappearedTN"][map_name][query_time]
            )
            data["ChangePrecision"][map_name][query_time] = prec
            data["ChangeRecall"][map_name][query_time] = rec
            data["ChangeF1"][map_name][query_time] = f1

            prec, rec, f1 = compute_metrics_single(
                tp + tp2, fp + fp2 + hp + hp2, fn + fn2 + mn + mn2
            )
            data["ChangePrecisionIncl"][map_name][query_time] = prec
            data["ChangeRecallIncl"][map_name][query_time] = rec
            data["ChangeF1Incl"][map_name][query_time] = f1

            tp = data["NumObjDetected"][map_name][query_time]
            fp = data["NumObjHallucinated"][map_name][query_time]
            fn = data["NumObjMissed"][map_name][query_time]
            prec, rec, f1 = compute_metrics_single(tp, fp, fn)
            data["ObjectPrecision"][map_name][query_time] = prec
            data["ObjectRecall"][map_name][query_time] = rec
            data["ObjectF1"][map_name][query_time] = f1


def augment_dynamic_metrics(data):
    """
    Comptue all derived metrics such as precision, recall, and f1 score.
    data[metric][map_name][query_time]=values
    """
    new_metrics = ["DynamicPrecision", "DynamicRecall", "DynamicF1"]

    header = data["DynamicNumGtLoaded"]
    for metric in new_metrics:
        data[metric] = {}
        for map_name in header.keys():
            data[metric][map_name] = {}

    for map_name, query_times in header.items():
        for query_time in query_times.keys():
            tp = data["DynamicTP"][map_name][query_time]
            fp = data["DynamicFP"][map_name][query_time]
            fn = data["DynamicFN"][map_name][query_time]
            prec, rec, f1 = compute_metrics_single(tp, fp, fn)
            data["DynamicPrecision"][map_name][query_time] = prec
            data["DynamicRecall"][map_name][query_time] = rec
            data["DynamicF1"][map_name][query_time] = f1


def augment_bg_metrics(data):
    """
    Comptue all derived metrics such as precision, recall, and f1 score.
    data[metric][map_name][query_time]=values
    """
    thresholds = set()
    for metric in data.keys():
        if "@" in metric:
            thresholds.add(metric.split("@")[1])

    new_metrics = [f"F1@{t}" for t in thresholds]

    header = data["NumGTFailed"]
    for metric in new_metrics:
        data[metric] = {}
        for map_name in header.keys():
            data[metric][map_name] = {}

    for map_name, query_times in header.items():
        for query_time in query_times.keys():
            for t in thresholds:
                precision = data[f"Accuracy@{t}"][map_name][query_time]
                recall = data[f"Completeness@{t}"][map_name][query_time]
                denominator = precision + recall
                f1 = (
                    np.nan if denominator == 0 else 2 * precision * recall / denominator
                )
                data[f"F1@{t}"][map_name][query_time] = f1


"""
Tools to computederived metrics.
"""


def compute_metrics(tp, fp, fn):
    """
    Returns (precision, recall, f1 score) for arrays
    """
    denominator = tp + fp
    denominator[denominator == 0] = np.nan
    precision = tp / denominator

    denominator = tp + fn
    denominator[denominator == 0] = np.nan
    recall = tp / denominator

    denominator = precision + recall
    denominator[denominator == 0] = np.nan
    f1 = 2 * precision * recall / denominator

    return precision, recall, f1


def compute_metrics_single(tp, fp, fn):
    """
    Returns (precision, recall, f1 score) for individual values
    """
    denominator = tp + fp
    precision = np.nan if denominator == 0 else tp / denominator

    denominator = tp + fn
    recall = np.nan if denominator == 0 else tp / denominator

    denominator = precision + recall
    f1 = np.nan if denominator == 0 else 2 * precision * recall / denominator

    return precision, recall, f1


def read_object_presence_times(filename):
    # Presence data[object_id][field] = values (vector)
    fields = [
        "observed",
        "estimated",
        "first_seen",
        "last_seen",
        "cd_before_present",
        "cd_before_absent",
        "cd_after_present",
        "cd_after_absent",
    ]
    data = {}
    with open(filename, "r") as file:
        reader = csv.reader(file)
        reader.__next__()  # Skip header
        for row in reader:
            object_id = object_string_to_id(row[0])
            values = {
                f: values_string_to_array(row[i + 1]) for i, f in enumerate(fields)
            }
            data[object_id] = values
    return data


def read_associations(filename):
    # associations[map_id][timestamp][from_id] = to_id or None
    gt_to_dsg = {}
    dsg_to_gt = {}
    with open(filename, "r") as file:
        reader = csv.reader(file)
        reader.__next__()  # Skip header
        for row in reader:
            map_id = int(row[0])
            query_time = int(row[1])
            from_gt = int(row[2]) == 1
            from_id = object_string_to_id(row[3])
            to_id = None if row[4] == "Failed" else object_string_to_id(row[4])

            # Parse down the dict tree to insert the value.
            storage = gt_to_dsg if from_gt else dsg_to_gt
            if not map_id in storage:
                storage[map_id] = {}
            storage = storage[map_id]
            if not query_time in storage:
                storage[query_time] = {}
            storage[query_time][from_id] = to_id
    return gt_to_dsg, dsg_to_gt


def object_string_to_id(object_string):
    """
    Convert an object string, assumed to be 'O(id)' to an int id.
    """
    if len(object_string) < 4:
        print(f"Can't convert object id string '{object_string}' to int.")
        return None
    return int(object_string[2:-1])


def values_string_to_array(values_string):
    """
    Convert a values string, assumed to be 'v1:v2:v3:v4' to an array of ints.
    """
    if values_string == "":
        return np.array([])
    return np.array([np.uint64(x) for x in values_string.split(":")])


def read_timing_stats(filename):
    """
    Read a stats.csv file. data = {timer_name: {mean, min, max, std}} with values in seconds.
    """
    data = {}
    with open(filename, "r") as file:
        csv_reader = csv.reader(file)
        csv_reader.__next__()  # Skip header
        for row in csv_reader:
            data[row[0]] = {
                "mean": float(row[1]),
                "min": float(row[2]),
                "max": float(row[3]),
                "std": float(row[4]),
            }
    return data


def read_timing_raw(filename):
    """
    Read a *_timing_raw.csv file. data = {stamps, duration} as arrays and duration in seconds.
    """
    x = []
    y = []
    with open(filename, "r") as file:
        reader = csv.reader(file)
        reader.__next__()  # Skip header
        for row in reader:
            x.append(np.uint64(row[0]))
            y.append(float(row[1]))
    return {"stamps": np.array(x), "duration": np.array(y)}


def read_change_detection_timings(exp_dir):
    """
    Read all change detection data from the experiment directory.
    data = {change_detector: {timer_name: times }}, where times is a list (in order of sorted map names) in seconds.
    """
    file_identifier = "_change_detection_timing.csv"
    methods = detect_methods(exp_dir)
    data = {}
    for detector in set([m[0] for m in methods]):
        local_data = {}
        dir = os.path.join(exp_dir, "pipeline", detector)
        files = [f for f in os.listdir(dir) if f.endswith(file_identifier)]
        for f in files:
            with open(os.path.join(dir, f), "r") as file:
                csv_reader = csv.reader(file)
                csv_reader.__next__()  # Skip header
                for row in csv_reader:
                    if row[0] not in local_data:
                        local_data[row[0]] = [float(row[1])]
                    else:
                        local_data[row[0]].append(float(row[1]))
        for d in local_data:
            local_data[d] = np.array(local_data[d])
        data[detector] = local_data
    return data


def read_reconciliation_timings(exp_dir):
    """
    Read all reconciliation data from the experiment directory.
    data = {change_detector: {reconciler: {timer_name: times }}}, where times is a list (in order of sorted map names) in seconds.
    """
    file_identifier = "_reconciliation_timing.csv"
    methods = detect_methods(exp_dir)
    data = {}
    for cd, rec in methods:
        if cd not in data:
            data[cd] = {}
        local_data = {}
        dir = os.path.join(exp_dir, "pipeline", cd, rec)
        files = [f for f in os.listdir(dir) if f.endswith(file_identifier)]
        for f in files:
            with open(os.path.join(dir, f), "r") as file:
                csv_reader = csv.reader(file)
                csv_reader.__next__()  # Skip header
                for row in csv_reader:
                    if row[0] not in local_data:
                        local_data[row[0]] = [float(row[1])]
                    else:
                        local_data[row[0]].append(float(row[1]))
        for d in local_data:
            local_data[d] = np.array(local_data[d])
        data[cd][rec] = local_data
    return data
