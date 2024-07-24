import time
import argparse
import yaml
import os
import subprocess
import shutil
import signal
from utils import (
    read_timestamps,
    get_4d_data_slice,
    augment_object_metrics,
    read_map_names,
    parse_table_data,
)
from tables import table


class FrontendExperimentConfig:
    def __init__(
        self,
        output_folder,
        bag_file,
        bag_duration,
        rate=0.5,
        num_trials=1,
        overwrite=False,
        shutdown_time=30,
    ):
        self.bag_file = bag_file
        self.output_folder = output_folder
        self.num_trials = num_trials
        self.overwrite = overwrite
        self.tfs = ["gt_poses", "kimera"]
        self.frontends = ["gt_seg", "oneformer", "openset"]
        self.rate = rate
        self.experiment_duration = bag_duration / rate + 60  # buffer
        self.start_visualizer = False
        self.shutdown_time = shutdown_time


def run_experiment(config):
    for trial in range(config.num_trials):
        for tf in config.tfs:
            for frontend in config.frontends:
                path = "{}/trial_{}/{}/{}".format(
                    config.output_folder, trial, tf, frontend
                )
                if not config.overwrite:
                    if os.path.exists(path):
                        print("{} exists. Skipping...".format(path))
                        continue

                print(
                    "Running trial {} with tf input {} and {} as frontend. ".format(
                        trial, tf, frontend
                    )
                )
                output_arg = "output_dir:={}".format(path)
                bag_arg = "bag_path:={}".format(config.bag_file)

                dataset_arg = "dataset:=apartment"

                openset_arg = "use_prerecorded_objects:={}".format(
                    str(frontend == "openset").lower()
                )
                oneformer_arg = "use_oneformer:={}".format(
                    str(frontend == "oneformer").lower()
                )

                gt_pose_arg = "use_gt_frame:={}".format(str(tf == "gt_poses").lower())

                rate_arg = "play_rate:={}".format(config.rate)

                visualizer_arg = "start_visualizer:={}".format(
                    str(config.start_visualizer).lower()
                )

                process = [
                    "roslaunch",
                    "khronos_ros",
                    "uhumans2_khronos.launch",
                    output_arg,
                    bag_arg,
                    openset_arg,
                    oneformer_arg,
                    gt_pose_arg,
                    rate_arg,
                    dataset_arg,
                    visualizer_arg,
                ]
                pro_env = os.environ.copy()

                pro = subprocess.Popen(
                    process,
                    stdout=subprocess.PIPE,
                    shell=False,
                    preexec_fn=os.setsid,
                    env=pro_env,
                )

                time.sleep(config.experiment_duration)
                os.killpg(os.getpgid(pro.pid), signal.SIGTERM)

                subprocess.Popen(["rosnode", "kill", "-a"])
                time.sleep(config.shutdown_time)


def run_evaluation(config):
    eval_path = subprocess.check_output(["rospack", "find", "khronos_eval"]).decode(
        "utf-8"
    )[:-1]
    eval_script = os.path.join(eval_path, "scripts/evaluate_pipeline.sh")
    for trial in range(config.num_trials):
        for tf in config.tfs:
            for frontend in config.frontends:
                path = "{}/trial_{}/{}/{}".format(
                    config.output_folder, trial, tf, frontend
                )
                eval_config = os.path.join(eval_path, "config/pipeline/apartment.yaml")
                if not config.overwrite:
                    if os.path.exists(path + "/results"):
                        print("{}/results exists. Skipping...".format(path))
                        continue
                print(
                    "Evaluating trial {} with tf input {} and {} as frontend. ".format(
                        trial, tf, frontend
                    )
                )
                eval_p = subprocess.run(
                    [eval_script, path, eval_config],
                    stdout=subprocess.PIPE,
                    shell=False,
                    preexec_fn=os.setsid,
                )
                # eval_p.wait()


def generate_table(config):
    metrics = ["ChangePrecision", "ChangeRecall", "ChangeF1"]
    methods = []
    method_labels = []
    for trial in range(config.num_trials):
        for tf in config.tfs:
            for frontend in config.frontends:
                path = "trial_{}/{}/{}".format(trial, tf, frontend)
                methods.append(os.path.join(path, "results/Middle/NotMerged"))
                method_labels.append("{}-{}-{}".format(trial, tf, frontend))

    data = parse_table_data(config.output_folder, methods)
    # Get common data
    timestamps = [
        read_timestamps(os.path.join(config.output_folder, m).split("/results/")[0])
        for m in methods
    ]
    map_names = [
        [
            int(x)
            for x in read_map_names(
                os.path.join(config.output_folder, m).split("/results/")[0]
            )
        ]
        for m in methods
    ]
    for d in data:
        augment_object_metrics(d)
    print(len(data))
    print(metrics)
    table(data, map_names, timestamps, method_labels, metrics)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Khronos Frontend Experiment.")
    parser.add_argument("output_folder", type=str, help="Result log folder.")
    parser.add_argument("bag_file", type=str, help="Input Bag.")
    parser.add_argument("bag_duration", type=float, help="Rosbag duration.")

    parser.add_argument("--overwrite", default=False)
    parser.add_argument("--rate", "-r", type=float, default=0.5)
    parser.add_argument("--trials", "-t", type=int, default=1)

    args = parser.parse_args()

    config = FrontendExperimentConfig(
        args.output_folder,
        args.bag_file,
        args.bag_duration,
        args.rate,
        args.trials,
        args.overwrite,
    )

    run_experiment(config)
    run_evaluation(config)
    generate_table(config)
