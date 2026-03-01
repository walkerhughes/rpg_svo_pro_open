# Stereo + IMU Configuration Guide

This document walks through configuring SVO with a stereo camera and IMU, using the FLA dataset as an example. The FLA dataset (`fla_stereo_imu`) contains synchronized stereo images and IMU measurements.

## Calibration File

### Stereo cameras

The calibration file for stereo cameras (e.g. `svo_ros/param/calib/fla_stereo_imu.yaml`) specifies two cameras:

```yaml
cameras:
- camera:
    <distortion and intrinsics>
  T_B_C:
    cols: 4
    rows: 4
    data: [0.99984983, -0.00373198,  0.01692284, -0.074783895,
           0.00403133,  0.9998354 , -0.01768969, -0.0005,
          -0.01685403,  0.01775525,  0.9997003 , -0.0041,
           0.        ,  0.        ,  0.        ,  1.        ]
    <......>
- camera:
    <distortion and intrinsics>
  T_B_C:
    cols: 4
    rows: 4
    data: [0.99746136, -0.00394329, -0.07110054,  0.074783895,
           0.00236645,  0.99974967, -0.02224833,  0.0005,
           0.07117047,  0.02202359,  0.997221  ,  0.0041,
           0.        ,  0.        ,  0.        ,  1.       ]
  <......>
label: fla_forward_stereo
```

`T_B_C` is the transformation of each camera in the body frame. The body frame is typically defined as the IMU frame (see notation [here](https://github.com/uzh-rpg/rpg_svo/wiki/Notation)).

### IMU parameters

IMU noise characteristics and initial values are specified in the calibration file:

```yaml
imu_params:
  delay_imu_cam: 0.0
  max_imu_delta_t: 0.01
  acc_max: 176.0
  omega_max: 7.8
  sigma_omega_c: 0.005
  sigma_acc_c: 0.01
  sigma_omega_bias_c: 4.0e-6
  sigma_acc_bias_c: 0.0002
  sigma_integration: 0.0
  g: 9.81007
  imu_rate: 200.0

imu_initialization:
  velocity: [0, 0, 0]
  omega_bias: [0, 0, 0]
  acc_bias: [0, 0, 0]
  velocity_sigma: 0.5
  omega_bias_sigma: 0.005
  acc_bias_sigma: 0.05
```

The most important noise parameters are:
* `sigma_omega_c` -- continuous-time gyroscope additive noise sigma
* `sigma_acc_c` -- continuous-time accelerometer additive noise sigma
* `sigma_omega_bias_c` -- continuous-time gyroscope bias random walk sigma
* `sigma_acc_bias_c` -- continuous-time accelerometer bias random walk sigma

For a detailed description of the IMU noise model, see [this page](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model).

To calibrate the IMU-camera extrinsics and intrinsics, use [Kalibr](https://github.com/ethz-asl/kalibr). A conversion script is provided at `svo_ros/scripts/kalibr_to_svo.py`.

**Tip:** The translation components between IMU and cameras are sometimes poorly calibrated, likely due to insufficient accelerometer excitation. Double-check your calibration before use.

## Pipeline Parameters

### Stereo

To use stereo, set the following in your parameter file:

```yaml
pipeline_is_stereo: True
automatic_reinitialization: True  # stereo can recover immediately when lost
```

Important depth parameters affecting epipolar search range for triangulation:

```yaml
max_depth_inv: 0.05
min_depth_inv: 2.0
mean_depth_inv: 0.3
```

These defaults work for most scenarios, but adjust them if your application has a very different depth range.

For multi-threaded feature reprojection in stereo:

```yaml
use_async_reprojectors: True
```

### IMU

To use IMU measurements for rotational priors:

```yaml
use_imu: True
img_align_prior_lambda_rot: 5.0   # gyroscope prior in sparse image alignment
poseoptim_prior_lambda: 2.0       # gyroscope prior in pose optimization
```

Higher prior lambdas give more weight to IMU priors. For a high-quality IMU, set them higher; for a noisy IMU, set them lower.

**Important:** If `use_imu` is `False` but the prior lambdas are nonzero, a constant velocity prior is used, which often does not perform well. Set prior lambdas to zero when IMU is not used.

### Image resolution

For higher-resolution images (e.g. 1280x1040 in the FLA dataset vs. the default 752x480), adjust these parameters:

```yaml
img_align_max_level: 5   # more pyramid levels for coarse-to-fine optimization
n_pyr_levels: 4          # always img_align_max_level minus one

grid_size: 50            # larger grid for feature bucketing
poseoptim_thresh: 3.0    # higher outlier threshold (in pixels) for larger images

init_min_disparity: 30   # higher minimum disparity for monocular initialization
```

## C++ API Usage

```cpp
#include <svo/svo_factory.h>
#include <yaml-cpp/yaml.h>

YAML::Node config = YAML::LoadFile("fla_stereo_imu.yaml");
auto camera = loadCameraBundle(config);
auto svo = svo::factory::makeStereo(config, camera);

// Feed images and IMU data to the pipeline
// svo->addImageBundle(images, timestamp);
// svo->addImuMeasurement(acc, gyro, timestamp);
```
