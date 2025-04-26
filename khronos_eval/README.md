# Khrons Eval
Evaluation suite for `khronos`.

Download ground-truth files to evaluate relevant datasets [here](https://drive.google.com/drive/folders/1lKbbpb1gWwrGBcQLikoaavXIO41fHOJ9?usp=drive_link).

> The most relevant file is `scripts/evaluate_pipeline.sh`, following that script should point you at all relevant files (mostly `src/pipeline_evaluator.cpp` that runs all evaluations).

After setting the script parameters and associated config file parameters properly, run the script to generate the `results` subdirectory with all relevant evaluation results in the output directory. To get a quick overview of the results, run `python3 plotting/tables.py`. Note that you will have to manually set filepaths and provide a list of `method`s, which are all of the runs you want to evaluate over, in that file as well. 

Once you run [tables.py](khronos_eval/plotting/tables.py), you should see something like this (with rows for every dataset you're evaluating):

| Data      | Accuracy@0.2 | Completeness@0.2 | F1@0.2 | ObjectPrecision | ObjectRecall | ObjectF1 | DynamicPrecision | DynamicRecall | DynamicF1 | ChangePrecision | ChangeRecall | ChangeF1 |
|-----------|--------------|------------------|--------|-----------------|--------------|----------|------------------|---------------|-----------|-----------------|--------------|----------|
| apartment | 99.9         | 91.9             | 95.5   | 95.7            | 37.0         | 53.1     | 100.0            | 33.2          | 49.5      | 25.7            | 60.4         | 47.8     |
| office    | 99.3         | 77.0             | 84.1   | 98.6            | 43.3         | 54.8     | 98.7             | 26.6          | 41.4      | 34.7            | 49.3         | 51.7     |


You can change output format in `tables.py`, with parameter `PRINT_MODE`.

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
