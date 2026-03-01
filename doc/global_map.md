# Using SVO with iSAM2-Based Global Map

As described in [SVO + OKVIS backend](./vio.md), fixed-lag smoothing is good for constant-time updates but inevitably drifts over time. Loop closure + pose graph optimization helps maintain global consistency, but is limited in accuracy because it cannot consider raw sensor measurements.

Therefore, we integrated a globally bundle-adjusted map using iSAM2. In short, once a keyframe exits the sliding window of the Ceres backend, we add it to a global visual-inertial bundle adjustment managed by iSAM2, which runs in a separate thread. In the visual front-end, we track points already optimized by iSAM2 (assuming iSAM2 converges fast enough to offer better accuracy).

The motivation is to achieve high real-time accuracy by localizing against a globally consistent map at every frame, instead of relying on non-causal information to correct drift. The matching of the current frame against the global map uses the same approach as SVO -- direct tracking using patches. This is not very robust, but works quite well in mild motion (probably thanks to the gyroscopes).

**Implementation detail:** To handle occasionally indeterministic errors with iSAM2, a weak prior on landmark depth is added when a landmark enters the global map (assuming the Ceres backend provides a reasonable initial estimate).

**Note:** The global map function has only been tested with monocular configuration and is relatively less stable than the visual-inertial odometry part.

## Prerequisites

Building with global map support requires GTSAM 4.2. Since GTSAM is incompatible with Eigen 5, it must be built from source with its bundled Eigen. See the [main README](../README.md#building-with-global-map-gtsam) for full build instructions.

Build SVO with the global map enabled:

```sh
cmake .. -DCMAKE_BUILD_TYPE=Release -DSVO_BUILD_GLOBAL_MAP=ON \
  -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM
cmake --build . -j$(nproc)
```

## Configuration

The global map is configured via `svo_ros/param/global_map.yaml` in addition to the standard VIO parameters.

### C++ API usage

```cpp
#include <svo/svo_factory.h>
#include <yaml-cpp/yaml.h>

YAML::Node config = YAML::LoadFile("global_map.yaml");
auto camera = loadCameraBundle(config);
auto svo = svo::factory::makeMono(config, camera);
```

## Pipeline Architecture

The global map pipeline extends the VIO pipeline:

1. **Visual front-end (SVO):** Tracks features semi-directly, creates keyframes.
2. **Sliding window backend (Ceres):** Optimizes recent keyframes in a fixed-size window.
3. **Global map (iSAM2):** Keyframes that exit the sliding window are added to iSAM2's incremental factor graph. iSAM2 performs global bundle adjustment in a background thread.
4. **Re-localization:** When the camera revisits a previously mapped region, the front-end matches against iSAM2-optimized landmarks, anchoring the sliding window to the globally consistent map.

### Visualization cues

* **Green points** -- landmarks in the current sliding window optimization, with trailing lines showing the active keyframes.
* **Blue points** -- landmarks optimized and managed by iSAM2. Points exiting the sliding window transition from green to blue. iSAM2-optimized points are slightly more accurate.
* **Golden lines** -- matches between the current frame and iSAM2 landmarks during re-localization.
* **Green rectangle around image** -- indicates enough fixed landmarks from the global map are observed in the current sliding window, allowing the pipeline to drop the usual first-frame fixation constraint.

## EuRoC Example

Using the EuRoC MH_03_medium sequence with a monocular global map configuration:

![](./images/mh03_gm.gif)

As the camera moves back to previously mapped regions, the pipeline matches features against the blue (iSAM2) points. Successfully matched landmarks serve as fixed anchors in the sliding window optimization, anchoring the local estimate to the global map.
