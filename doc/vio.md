# Visual-Inertial Odometry with SVO Front-End

The visual-inertial odometry (VIO) consists of the SVO 2.0 visual front-end and an OKVIS-style sliding window optimization backend (Ceres-based). Both monocular and stereo setups are supported.

Loop closure and correction using DBoW2 is also supported. The loop closure is conservative: only very good matches after 2D-2D/2D-3D verification plus large pose correction are accepted. ORB features are additionally extracted for loop closure, and their depths are estimated using depth filters.

## Configuration

VIO is configured through YAML parameter and calibration files:

* **Monocular VIO parameters:** `svo_ros/param/vio_mono.yaml`
* **Stereo VIO parameters:** `svo_ros/param/vio_stereo.yaml`
* **Monocular calibration (EuRoC):** `svo_ros/param/calib/euroc_mono.yaml`
* **Stereo calibration (EuRoC):** `svo_ros/param/calib/euroc_stereo.yaml`

The mono and stereo parameter files are essentially identical, except for the `pipeline_is_stereo` flag.

### C++ API usage

```cpp
#include <svo/svo_factory.h>
#include <yaml-cpp/yaml.h>

YAML::Node config = YAML::LoadFile("vio_mono.yaml");
auto camera = loadCameraBundle(config);

// For monocular VIO:
auto svo = svo::factory::makeMono(config, camera);

// For stereo VIO:
auto svo = svo::factory::makeStereo(config, camera);
```

## Pipeline Overview

The VIO pipeline works as follows:

1. **Visual front-end (SVO):** Tracks features frame-to-frame using semi-direct alignment. Creates keyframes and manages depth filters.

2. **Sliding window backend (Ceres):** When a new keyframe is created, it is added to the sliding window optimization. The backend jointly optimizes poses, velocities, IMU biases, and 3D landmark positions using visual reprojection errors and IMU preintegration factors.

3. **Loop closure (optional):** ORB features extracted from keyframes are matched against a DBoW2 vocabulary database. Successful loop closures are integrated as constraints in pose graph optimization or the global bundle adjustment.

### Visualization cues

If you visualize the output:
* **Green points** -- landmarks optimized by the backend, fed back to the front-end for tracking. Brighter colors indicate newer landmarks.
* **Line segments** connecting the current frame to past keyframes indicate loop closures:
  * **Red lines** -- successful recognition
  * **Green lines** -- actual pose correction applied

## Key Parameters

### Backend parameters (`backend.yaml` / `vio_mono.yaml`)

| Parameter | Description |
|---|---|
| `pipeline_is_stereo` | `True` for stereo, `False` for monocular |
| `use_imu` | Enable IMU integration |
| `backend_type` | Backend type (0: none, 1: Ceres sliding window) |

### Loop closure parameters (in VIO parameter files)

| Parameter | Description |
|---|---|
| `runlc` | Enable loop closure (requires DBoW2 vocabulary) |
| `lc_min_inlier_count` | Minimum inlier matches for loop acceptance |

### Loop closure vocabulary

Download the DBoW2 vocabulary files before enabling loop closure:

```sh
cd svo_online_loopclosing/vocabularies && ./download_voc.sh
```

Without the vocabulary files, the pipeline will crash on initialization.

## EuRoC Dataset Tips

For monocular VIO, start playback at a later timestamp to avoid poor initialization conditions (strong rotation at the start of many sequences). See the [visual front-end guide](./frontend/visual_frontend.md) for recommended start offsets per sequence.
