# Khrons Eval
Evaluation suite for `khronos`.

## Current output directory structure
The **core** structure is governed by the [experiment_manager](https://github.mit.edu/SPARK/Khronos/blob/feature/eval_reconciliation/khronos_ros/include/khronos_ros/experiments/experiment_manager.h) and created when running `khronos.launch`:
```bash
├── config.txt      # Human readable config of khronos that was run.
├── experiment_log.txt  # Meta iformaton about the experiment and future evaluations. This file should say "[FLAG] [Experiment Finished Cleanly]" somewhere if the data is complete.
├── maps        # Periodically saved states of khronos for further evaluation.
│   ├── 00000
│   │   ├── backend
│   │   │   ├── dsg.sparkdsg
│   │   │   ├── dsg_with_mesh.sparkdsg
│   │   │   ├── loop_closures.csv
│   │   │   └── mesh.ply
│   │   └── timestamp.txt
│   ├── 00001
│    ...
├── metrics     # Output of various evaluation tasks, such as mesh evaluation.
│   ├── mesh.csv
│   ...
├── plots   # Outputs of scripts that plot the results from 'metrics'
│   ├── mesh_metrics.png
│   ...
├── reconciliation  # directory where all additional data for reconciliation evaluation is stored.
│   └──
├── rosparams.yaml  # The ros params that were used when running khronos, these can be used to recreate an experiment.
└── timing      # Timing information when khronos was run.
    └── stats.csv
```
