# Khronos

![Khronos](https://github.com/MIT-SPARK/Khronos/assets/36043993/a3f63175-48cc-4ef2-a810-19e4e5cd360c)

Khronos is a unified approach that can reason about short-term dynamics and long-term changes when performing online metric-semantic simultaneous mapping and localization (SLAM) in dynamic environments. A few instances from Khronos’ spatio-temporal map, representing the scene state at all times, are shown above. Short-term dynamics (left) are shown in magenta and compared against observed human actions over the corresponding time interval. Both humans and inanimate objects (the cart) are detected. Long-term changes (right) are shown for three time instances of the same scene. The earliest instance is at time 0:20 (top right). While the robot is moving through the hallways, a chair is removed and a red cooler is placed on top of the table; these changes are detected as the robot revisits and closes the loop at time 1:52 (bottom right). Lastly, the cooler is removed again, which is detected by the robot at time 3:35.

> This project was supported by Amazon, the ARL DCIST program, the ONR RAPID program, and the Swiss National Science Foundation (SNSF) grant No. 214489.

# Table of Contents
**Credits**
* [Paper](#Paper)
* [Video](#Video)

**Setup**
* [Installation](#Installation)
* [Dataset](#download-datasets)

**Examples**
* [Run Khronos](#run-khronos)
* [Visualize 4D Map](#visualize-4d-map)

**Contributing**
* [Contributing](#contributing)


# News
* [07/2024] Khronos won the **Outstanding Systems Paper Award** at RSS in Delft!

# Paper

If you use this code in your work, please cite the following paper:

Lukas Schmid, Marcus Abate, Yun Chang, and Luca Carlone, "**Khronos: A Unified Approach for Spatio-Temporal Metric-Semantic SLAM in Dynamic Environments**", in *Robotics: Science and Systems*, 2024. [ [ArXiv](https://arxiv.org/abs/2402.13817) | [Video](https://www.youtube.com/watch?v=YsH6YIL5_kc) ]

```bibtex
@inproceedings{Schmid2024Khronos,
title={Khronos: A Unified Approach for Spatio-Temporal Metric-Semantic SLAM in Dynamic Environments},
author={Lukas Schmid and Marcus Abate and Yun Chang and Luca Carlone},
year={2024},
booktitle = "Proc. of Robotics: Science and Systems",
}
```

# Video
An overview of Khronos is available on [YouTube](https://www.youtube.com/watch?v=YsH6YIL5_kc):

[<img src=https://github.com/MIT-SPARK/Khronos/assets/36043993/279fe8e8-60d9-44ee-af34-1947d4a1aeb0 alt="Khronos Youtube Video">](https://www.youtube.com/watch?v=YsH6YIL5_kc)

# Setup
## Installation
Setup a catkin workspace:
```bash
sudo apt install python3-catkin-tools python3-vcstool python3-tk
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/$ROS_DISTRO
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF
catkin config --merge-devel
```

Install system dependencies:
```bash
sudo apt install ros-$ROS_DISTRO-gtsam libgoogle-glog-dev nlohmann-json3-dev
```

Get Khronos and all source dependencies:
```bash
cd ~/catkin_ws/src
git clone git@github.com:MIT-SPARK/Khronos.git khronos
# Use `https.rosinstall` if you do not have ssh key setup with github.com.
vcs import . < khronos/install/ssh.rosinstall
```

Build:
```bash
catkin build khronos_ros
```

## Download datasets

Download the simulated datsets used in our paper [here](https://drive.google.com/drive/folders/1Miv2uPBYzg_FyqMX5vAWI4MDZ70sPLW-?usp=drive_link).

The `khronos_office.bag` dataset is the longer of the two and contains the most changes and dynamic objects.

Optionally, run `rosbag decompress khronos_office.bag` to unpack the rosbag for faster runtime.

# Examples
## Run Khronos

Open [uhumans2_khronos.launch](khronos_ros/launch/uhumans2_khronos.launch) and modify the bag path to the `tesse_cd` office rosbag.

Run the following in commandline:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch khronos_ros uhumans2_khronos.launch
```

RVIZ will launch automatically for real-time visualization. You should see the method build an incremental background mesh, detect and segment static and dynamic objects, and generate object nodes. The following image shows the human motion detection (left red), a segmented static chair object and associated scene-graph node (right blue), and a segmented dynamic object and associated node (center yellow):

<img src="https://github.com/MIT-SPARK/Khronos/assets/32229998/01c265ac-c2ab-42e7-8a6a-2f9df84fc85b" alt="Khronos Beginning" width="500"/>

Use the right side panel of rviz to toggle different visualization options to see each component of the method, including inpus, object/motion detection, factor graph, scene graph, and backend visuals. Most of these are disabled by default.

At the end of rosbag playback, Khronos will have a fully completed scene reconstruction with objects, dynamics, and changes:

<img src="https://github.com/MIT-SPARK/Khronos/assets/32229998/66aeae1e-bc96-42b7-bcc2-a94f268d4acd" alt="Khronos End" width="500">

After the rosbag has completed playback and the method is finished, call the following rosservice to guarantee that output is saved before termination:
```bash
rosservice call /khronos_node/experiment/finish_mapping_and_save
```

You can now terminate Khronos.

To run Khronos with the apartment dataset, change the `dataset` from `tesse_cd_office` to `tesse_cd_apartment` and change the rosbag path accordingly. You can then run the launch file as before.

## Visualize 4D map

To visualize the spatio-temporal map of the scene after running Khronos, run the following in commandline:

```bash
roslaunch khronos_ros 4d_visualizer.launch
```

The visualizer will load the map located (by default) in [output/tmp](output/tmp/). If you save the map elsewhere, be sure to modify the path in [4d_visualizer.launch](khronos_ros/launch/4d_visualizer.launch).

<img src="https://github.com/MIT-SPARK/Khronos/assets/32229998/d8072aab-a874-483f-aada-24607eb620be" alt="Khronos 4D Map" width="700">

You can play forward or backward, and see Khronos' estimate of the state of the scene at each timestamp, with current or future knowledge. As an example, fix the query time to 0 and play forward on robot time mode to see how Khronos' understanding of the initial state of the scene evolves over time.

# Contributing
This is an open-source project and pull requests for features and fixes are very welcome! We follow the [Feature-Branch-Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow) and the [google C++ style guide](https://google.github.io/styleguide/cppguide.html). To adhere to this, please setup the auto formatter and linter for khronos as follows:
```bash
roscd khronos
pip install pre-commit
pre-commit install
```
