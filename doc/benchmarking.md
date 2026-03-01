# Benchmarking

The original upstream repository included an `svo_benchmarking` package for batch evaluation of pose estimation accuracy across multiple datasets. That package depended on ROS infrastructure and has been removed from this standalone build.

## Evaluating Trajectory Accuracy

To evaluate SVO's trajectory estimation, you can use the [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation) package, which provides tools for computing standard metrics (ATE, RPE, etc.) from estimated and ground-truth trajectories.

## Running SVO on Datasets

To process a dataset (e.g. EuRoC), write a small C++ driver that:

1. Loads the YAML config and calibration files
2. Creates the SVO pipeline via the factory functions
3. Feeds images (and IMU measurements, if applicable) to the pipeline
4. Saves the estimated trajectory to a file

See the [main README](../README.md#configuration) for the C++ API and configuration details.
