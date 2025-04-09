# Khrons Eval
Evaluation suite for `khronos`.

> The most relevant file is `scripts/evaluate_pipeline.sh`, following that script should point you at all relevant files (mostly `src/pipeline_evaluator.cpp` that runs all evaluations).

After setting the script parameters and associated config file parameters properly, run the script to generate the `results` subdirectory with all relevant evaluation results in the output directory. To get a quick overview of the results, run `python3 plotting/tables.py`. Note that you will have to manually set filepaths and provide a list of `method`s, which are all of the runs you want to evaluate over, in that file as well. 

## Current output directory structure
The **core** structure is governed by the [experiment_manager](https://github.mit.edu/SPARK/Khronos/blob/feature/eval_reconciliation/khronos_ros/include/khronos_ros/experiments/experiment_manager.h) and created when running `khronos.launch`:
```bash
├── config.txt      # Human readable config of khronos that was run.
├── experiment_log.txt  # Meta iformaton about the experiment and future evaluations. This file should say "[FLAG] [Experiment Finished Cleanly]" somewhere if the data is complete.
├── maps        # Periodically saved states of khronos for further evaluation. This is OPTIONAL, activated by setting `save_every_n_frames` to > 0 in the config.
│   ├── 00000
│   │   ├── backend
│   │   │   ├── dsg.sparkdsg
│   │   │   ├── dsg_with_mesh.sparkdsg
│   │   │   ├── loop_closures.csv
│   │   └── timestamp.txt
│   ├── 00001
│    ...
├── final.4dmap  # The 4D map output from Khronos, to be evaluated.
├── rosparams.yaml  # The ros params that were used when running khronos, these can be used to recreate an experiment.
└── timing      # Timing information when khronos was run.
    └── stats.csv
    └── ...
```
