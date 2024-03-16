# Khronos
This repository will contain the code for **Khronos**, our framework for online Spatio-Temporal Metric-Semantic SLAM (SMS).

![Khronos](https://github.mit.edu/storage/user/26675/files/2c13cac1-32d1-40fc-a41b-b63f7ed2c050)

Khronos is a unified approach that can reason about short-term dynamics and long-term changes when performing online metric-semantic simultaneous mapping and localization (SLAM) in dynamic environments. A few instances from Khronos’ spatio-temporal map, representing the scene state at all times, are shown above. Short-term dynamics (left) are shown in magenta and compared against observed human actions over the corresponding time interval. Both humans and inanimate objects (the cart) are detected. Long-term changes (right) are shown for three time instances of the same scene. The earliest instance is at time 0:20 (top right). While the robot is moving through the hallways, a chair is removed and a red cooler is placed on top of the table; these changes are detected as the robot revisits and closes the loop at time 1:52 (bottom right). Lastly, the cooler is removed again, which is detected by the robot at time 3:35.

> This project was supported by the Amazon Science Hub “Next-Generation Spatial AI for Human-Centric Robotics” project, the ARL DCIST program, the ONR RAPID program, and the Swiss National Science Foundation (SNSF) grant No. 214489.

# Table of Contents
**Credits**
* [Paper](#Paper)
* [Video](#Video)   

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

* Lukas Schmid, Marcus Abate, Yun Chang, and Luca Carlone, "**Khronos: A Unified Approach for Spatio-Temporal Metric-Semantic SLAM in Dynamic Environments**", in *ArXiv Preprint*, 2024. [ [ArXiv](https://arxiv.org/abs/2402.13817) | [Video](https://www.youtube.com/watch?v=YsH6YIL5_kc) ]
  ```bibtex
   @misc{Schmid2024Khronos,
      title={Khronos: A Unified Approach for Spatio-Temporal Metric-Semantic SLAM in Dynamic Environments}, 
      author={Lukas Schmid and Marcus Abate and Yun Chang and Luca Carlone},
      year={2024},
      eprint={2402.13817},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
   }
  ```

# Video
An overview of Khronos is available on [YouTube](https://www.youtube.com/watch?v=YsH6YIL5_kc):

[<img src=https://github.com/MIT-SPARK/Khronos/assets/36043993/279fe8e8-60d9-44ee-af34-1947d4a1aeb0 alt="Khronos Youtube Video">](https://www.youtube.com/watch?v=YsH6YIL5_kc)
