# The Visual Front-End

## Overview

SVO's visual front-end handles feature tracking, depth estimation, and pose estimation using a semi-direct approach. It supports perspective and fisheye/catadioptric cameras in monocular or stereo configurations, with optional active exposure control.

## Configuration

SVO is configured via YAML files loaded through `YAML::Node`. Two types of YAML files are needed:

1. **Parameter file** -- pipeline settings (feature detection thresholds, pyramid levels, grid size, etc.)
2. **Calibration file** -- camera intrinsics, distortion, and optionally IMU extrinsics and noise model

Example parameter and calibration files are provided under `svo_ros/param/` and `svo_ros/param/calib/`.

### Using SVO in your application

```cpp
#include <svo/svo_factory.h>
#include <yaml-cpp/yaml.h>

// Load parameter and calibration YAML files
YAML::Node config = YAML::LoadFile("pinhole.yaml");
auto camera = loadCameraBundle(config);
auto svo = svo::factory::makeMono(config, camera);
```

## Parameter Files

We provide several example parameter files under `svo_ros/param/`:

* `pinhole.yaml` -- for relatively small field-of-view cameras (e.g. 752x480)
* `fisheye.yaml` -- for cameras with wide-angle lenses (fisheye and catadioptric)
* `frontend_imu/euroc_mono_imu.yaml` -- monocular + IMU (EuRoC dataset)
* `frontend_imu/euroc_stereo_imu.yaml` -- stereo + IMU (EuRoC dataset)

The parameters in these files are typical values. Refer to the comments in each file for descriptions and tuning guidance.

### Key parameters

```yaml
# Pyramid levels for coarse-to-fine optimization
img_align_max_level: 4    # maximum pyramid level
n_pyr_levels: 3           # total pyramid levels (typically max_level - 1)

# Feature grid
grid_size: 30             # feature bucketing grid cell size (pixels)

# Outlier rejection
poseoptim_thresh: 2.0     # pose optimization outlier threshold (pixels)

# Initialization (monocular only)
init_min_disparity: 25    # minimum disparity for initialization
```

For higher-resolution images (e.g. 1280x1040), increase pyramid levels, grid size, and thresholds proportionally. See [frontend_fla.md](./frontend_fla.md) for a worked example.

## Calibration Files

Calibration files define camera intrinsics, distortion model, and body-camera extrinsic transformations. Example calibration files are under `svo_ros/param/calib/`:

* `svo_test_pinhole.yaml` -- test pinhole camera
* `euroc_mono.yaml` -- EuRoC monocular
* `euroc_stereo.yaml` -- EuRoC stereo
* `bluefox_25000826_fisheye.yaml` -- Bluefox fisheye camera
* `fla_stereo_imu.yaml` -- stereo + IMU (FLA dataset)

See [calibration.md](../calibration.md) for details on supported camera models and calibration tools.

## Inertial-Aided Front-End

Without the heavier Ceres-based backend, the front-end can use IMU measurements to facilitate visual tracking. This provides a rotational prior for image alignment and pose optimization. It is not as accurate as tightly-coupled VIO, but is relatively lightweight.

Example configs for the EuRoC dataset:
* `svo_ros/param/frontend_imu/euroc_mono_imu.yaml`
* `svo_ros/param/frontend_imu/euroc_stereo_imu.yaml`

### IMU parameters

```yaml
use_imu: True
img_align_prior_lambda_rot: 5.0   # gyroscope prior in sparse image alignment
poseoptim_prior_lambda: 2.0       # gyroscope prior in pose optimization
```

The higher the prior lambdas, the more weight the IMU priors carry. If you have a high-quality IMU, set them higher; for a noisy IMU, set them lower.

**Important:** If `use_imu` is `False` and the prior lambdas are nonzero, a constant velocity prior is used instead, which often does not perform well. Set the prior lambdas to zero when IMU is not used.

### EuRoC initialization tips

The first several images of many EuRoC sequences contain strong rotation or otherwise poor conditions for initialization, especially for monocular setups. Start playback at a later timestamp for best results:

| Sequence | Recommended start offset |
|---|---|
| MH_01_easy | 50s |
| MH_02_easy | 45s |
| V1_01_easy | 0s (start from beginning) |
| V1_02_medium | 13s |
| V2_01_easy | 0s (start from beginning) |
| V2_02_medium | 13s |

See [frontend_fla.md](./frontend_fla.md) for a step-by-step guide on using SVO with stereo + IMU.
