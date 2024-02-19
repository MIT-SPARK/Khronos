# Khronos
This repository will contain the code for **Khronos**, our framework for online Spatio-temporal Metric-Semantic SLAM (SMS).

![Khronos](https://github.mit.edu/storage/user/26675/files/2c13cac1-32d1-40fc-a41b-b63f7ed2c050)

Khronos is a unified approach that can reason about short-term dynamics and long-term changes when performing online metric-semantic simultaneous mapping and localization (SLAM) in dynamic environments. A few instances from Khronos’ spatio-temporal map, representing the scene state at all times, are shown above. Short-term dynamics (left) are shown in magenta and compared against observed human actions over the corresponding time interval. Both humans and inanimate objects (the cart) are detected. Long-term changes (right) are shown for three time instances of the same scene. The earliest instance is at time 0:20 (top right). While the robot is moving through the hallways, a chair is removed and a red cooler is placed on top of the table; these changes are detected as the robot revisits and closes the loop at time 1:52 (bottom right). Lastly, the cooler is removed again, which is detected by the robot at time 3:35.

> This project was supported by the Amazon Science Hub “Next-Generation Spatial AI for Human-Centric Robotics” project, the ARL DCIST program, the ONR RAPID program, and the Swiss National Science Foundation (SNSF) grant No. 214489.

# Table of Contents
**Credits**
* [Paper](#Paper)
   

> **__Note__** The code will be released here shortly.
>
> **Setup**
> * Installation
> * Datasets
>
> **Examples**
> - Running Khronos
> - Visualizing the 4D map

# Paper
If you find this useful for your research, please consider citing our paper:

* Lukas Schmid, Marcus Abate, Yun Chang, and Luca Carlone, "**Khronos: A Unified Approach for Spatio-Temporal Metric-Semantic SLAM in Dynamic Environments**", in *ArXiv Preprint*, 2024.
  ```bibtex
  @inproceedings{Schmid2024Khronos,
    title={Khronos: A Unified Approach for Spatio-Temporal Metric-Semantic SLAM
           in Dynamic Environments},
    author={Schmid, Lukas and Abate, Marcus and Chang, Yun and Carlone, Luca},
    booktitle={ArXiv Preprint},
    year={2024},
    volume={},
    number={},
    pages={},
    doi={}
  }
  ```
