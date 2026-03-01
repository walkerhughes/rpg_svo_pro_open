# rpg_svo_pro

This repo includes **SVO Pro** which is the newest version of Semi-direct Visual Odometry (SVO) developed over the past few years at the Robotics and Perception Group (RPG).
SVO was born as a fast and versatile visual front-end as described in the [SVO paper (TRO-17)](http://rpg.ifi.uzh.ch/docs/TRO17_Forster-SVO.pdf). Since then, different extensions have been integrated through various research and industrial projects.
SVO Pro features the support of different [camera models](http://rpg.ifi.uzh.ch/docs/ICRA16_Zhang.pdf), [active exposure control](http://rpg.ifi.uzh.ch/docs/ICRA17_Zhang.pdf), a sliding window based backend, and global bundle adjustment with loop closure.

In summary, this repository offers the following functionalities:

* Visual-odometry: The most recent version of SVO that supports perspective and fisheye/catadioptric cameras in monocular or stereo setup. It also includes active exposure control.
* Visual-inertial odometry: SVO fronted + visual-inertial sliding window optimization backend (modified from [OKVIS](https://github.com/ethz-asl/okvis))
* Visual-inertial SLAM: SVO frontend + visual-inertial sliding window optimization backend +  globally bundle adjusted map (using [iSAM2](https://gtsam.org/)). The global map is updated in real-time, thanks to iSAM2, and used for localization at frame-rate.
* Visual-inertial SLAM with loop closure: Loop closures, via [DBoW2](https://github.com/dorian3d/DBoW2), are integrated in the global bundle adjustment. Pose graph optimization is also included as a lightweight replacement of the global bundle adjustment.

An example of the visual-inertial SLAM pipeline on EuRoC dataset is below (green points - sliding window; blue points - iSAM2 map):

![](./doc/images/v102_gm.gif)

SVO Pro and its extensions have been used to support various projects at RPG, such as our recent work on [multiple camera SLAM](http://rpg.ifi.uzh.ch/docs/ICRA20_Kuo.pdf), [voxel map for visual SLAM](http://rpg.ifi.uzh.ch/docs/ICRA20_Muglikar.pdf) and [the tight-coupling of global positional measurements into VIO](http://rpg.ifi.uzh.ch/docs/IROS20_Cioffi.pdf). We hope that the efforts we made can facilitate the research and applications of SLAM and spatial perception.

## License

The code is licensed under GPLv3. For commercial use, please contact `sdavide [at] ifi [dot] uzh [dot] ch`.

The visual-inertial backend is modified from OKVIS, and the license is retained at the beginning of the related files.

## Credits

If you use the code in the academic context, please cite:

* Christian Forster, Matia Pizzoli, Davide Scaramuzza. SVO: Fast Semi-Direct Monocular Visual Odometry. ICRA, 2014. [bibtex](./doc/bib/Forster14icra.bib)
* Christian Forster, Zichao Zhang, Michael Gassner, Manuel Werlberger, Davide Scaramuzza. SVO: Semi-Direct Visual Odometry for Monocular and Multi-Camera Systems. TRO, 2017. [bibtex](./doc/bib/Forster17tro.bib)

Additionally, please cite the following papers for the specific extensions you make use of:

* *Fisheye/catadioptric camera extension*: Zichao Zhang, Henri Rebecq, Christian Forster, Davide Scaramuzza. Benefit of Large Field-of-View Cameras for Visual Odometry. ICRA, 2016. [bibtex](./doc/bib/Zhang16icra.bib)
* *Brightness/exposure compensation*: Zichao Zhang, Christian Forster, Davide Scaramuzza. Active Exposure Control for Robust Visual Odometry in HDR Environments. ICRA, 2017. [bibtex](./doc/bib/Zhang17icra.bib)
* *Ceres-based optimization backend*: Stefan Leutenegger, Simon Lynen, Michael Bosse, Roland Siegwart, Paul Timothy Furgale. Keyframe-based visual–inertial odometry using nonlinear optimization. IJRR, 2015. [bibtex](./doc/bib/Leutenegger15ijrr.bib)
* *Global map powered by iSAM2*: Michael Kaess, Hordur Johannsson, Richard Roberts, Viorela Ila, John Leonard, Frank Dellaert. iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree. IJRR, 2012. [bibtex](./doc/bib/Kaess12ijrr.bib)
* *Loop closure*: Dorian Galvez-Lopez and Juan D. Tardos. Bags of Binary Words for Fast Place Recognition in Image Sequences. TRO, 2012. [bibtex](./doc/bib/Galvez12tro.bib)

Our recent publications that use SVO Pro are:

* *Multiple camera SLAM*: Juichung Kuo, Manasi Muglikar, Zichao Zhang, Davide Scaramuzza. Redesigning SLAM for Arbitrary Multi-Camera Systems. ICRA, 2020. [bibtex](./doc/bib/Kuo20icra.bib)
* *Voxel map for visual SLAM*: Manasi Muglikar, Zichao Zhang, Davide Scaramuzza. Voxel Map for Visual SLAM. ICRA, 2020. [bibtex](./doc/bib/Muglikar20icra.bib)
* *Tight-coupling of global positional measurements into VIO*: Giovanni Cioffi, Davide Scaramuzza. Tightly-coupled Fusion of Global Positional Measurements in Optimization-based Visual-Inertial Odometry. IROS, 2020. [bibtex](./doc/bib/Cioffi20iros.bib)

## Install

### Prerequisites

This fork has been refactored from Catkin/ROS to a standalone CMake build. **ROS is no longer required.**

You will need:

* CMake 3.16+
* C++17 compiler (GCC 7+, Clang 5+, Apple Clang 10+)
* Eigen3
* OpenCV
* Ceres Solver 2.2+
* glog, gflags
* yaml-cpp
* Boost (only if building with `SVO_BUILD_GLOBAL_MAP=ON`)
* Google Test (only if building with `SVO_BUILD_TESTS=ON`)

#### macOS (Homebrew)

```sh
brew install cmake eigen opencv ceres-solver glog gflags yaml-cpp googletest
```

#### Ubuntu 22.04+

```sh
sudo apt-get install cmake build-essential libeigen3-dev libopencv-dev \
  libceres-dev libgoogle-glog-dev libgflags-dev libyaml-cpp-dev \
  libgtest-dev libsuitesparse-dev
```

> **Note on Ceres version:** This build requires Ceres 2.2+ (which uses the `ceres::Manifold` API). Ubuntu 22.04's default `libceres-dev` package may be older. If so, build [Ceres from source](http://ceres-solver.org/installation.html).

### Clone and build

```sh
git clone https://github.com/walkerhughes/rpg_svo_pro_open.git
cd rpg_svo_pro_open
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
```

[opengv](https://github.com/laurentkneip/opengv) and [DBoW2](https://github.com/dorian3d/DBoW2) are automatically downloaded via CMake `FetchContent` if not found on the system. [minkindr](https://github.com/ethz-asl/minkindr), [fast_neon](https://github.com/uzh-rpg/fast_neon), and eigen_checks are vendored under `third_party/`.

### Build options

| Option | Default | Description |
|---|---|---|
| `SVO_BUILD_TESTS` | `OFF` | Build unit tests |
| `SVO_BUILD_EXAMPLES` | `ON` | Build EuRoC dataset example programs |
| `SVO_BUILD_LOOP_CLOSING` | `ON` | Build loop closing and pose graph optimization modules |
| `SVO_BUILD_GLOBAL_MAP` | `OFF` | Build global map modules (requires GTSAM) |

### Running tests

Build with tests enabled and run via CTest:

```sh
cmake .. -DCMAKE_BUILD_TYPE=Release -DSVO_BUILD_TESTS=ON
cmake --build . -j$(nproc)
ctest --output-on-failure
```

### Running examples

Three example programs demonstrate running SVO on the [EuRoC MAV dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets):

| Example | Pipeline | Config | Description |
|---|---|---|---|
| `euroc_mono_vo` | Visual front-end only | `pinhole.yaml` | Simplest: monocular VO, no IMU, no backend |
| `euroc_mono_vio` | Front-end + IMU + backend | `vio_mono.yaml` | Monocular VIO with Ceres optimization |
| `euroc_stereo_vio` | Front-end + IMU + backend | `vio_stereo.yaml` | Stereo VIO — best accuracy |

**Quick start** — run a single sequence:

```sh
# 1. Download one EuRoC sequence (~1 GB)
bash examples/download_euroc.sh /tmp/euroc V1_01_easy

# 2. Run stereo VIO (recommended)
./build/examples/euroc_stereo_vio --dataset_dir /tmp/euroc/V1_01_easy/mav0

# 3. Or run mono VO (simplest, no IMU needed)
./build/examples/euroc_mono_vo --dataset_dir /tmp/euroc/V1_01_easy/mav0
```

Each program writes a TUM-format trajectory to `traj_estimate.txt` (configurable via `--output`). Use `--help` to see all options (e.g. `--config`, `--calib`, `--start_offset_sec`). To disable example builds: `-DSVO_BUILD_EXAMPLES=OFF`.

### Reproducing the benchmark table

To reproduce the full benchmark table below on your own machine:

```sh
# 1. Build (if not already done)
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
cd ..

# 2. Install the evaluation tool
pip install evo

# 3. Download all 11 EuRoC sequences (~25 GB total)
bash examples/download_euroc.sh /tmp/euroc ALL

# 4. Run all pipelines and evaluate (takes ~15 minutes)
bash examples/run_all_benchmarks.sh build /tmp/euroc /tmp/euroc_results
```

The benchmark script will:
1. Convert all ground truth files to TUM format
2. Run all 3 pipelines (`euroc_mono_vo`, `euroc_mono_vio`, `euroc_stereo_vio`) on all 11 sequences
3. Evaluate each trajectory against ground truth using `evo_ape --align --correct_scale`
4. Print a results table

To re-run a single pipeline or re-evaluate without re-running:

```sh
# Run one pipeline on one sequence
./build/examples/euroc_stereo_vio \
  --dataset_dir /tmp/euroc/MH_01_easy/mav0 \
  --output /tmp/euroc_results/MH_01_easy_euroc_stereo_vio.txt

# Evaluate against ground truth (convert GT first if needed)
evo_ape tum /tmp/euroc/MH_01_easy/mav0/groundtruth_tum.txt \
  /tmp/euroc_results/MH_01_easy_euroc_stereo_vio.txt \
  --align --correct_scale
```

### EuRoC benchmark results

ATE RMSE (meters) on all 11 [EuRoC MAV](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) sequences, evaluated with `evo_ape --align --correct_scale` (Sim(3) Umeyama alignment):

| Sequence | Mono VO | Mono VIO | Stereo VIO |
|---|---|---|---|
| MH_01_easy | 1.115 | 3.408 | 0.097 |
| MH_02_easy | 1.135 | 3.847 | 0.063 |
| MH_03_medium | 2.839 | 3.330 | 0.090 |
| MH_04_difficult | 5.436 | 6.505 | 0.125 |
| MH_05_difficult | 3.101 | 6.584 | 0.132 |
| V1_01_easy | 1.588 | 1.787 | 0.055 |
| V1_02_medium | 1.642 | 1.730 | 0.075 |
| V1_03_difficult | 1.503 | 0.145 | 0.059 |
| V2_01_easy | 1.918 | -- | 0.092 |
| V2_02_medium | 1.878 | 0.432 | 0.109 |
| V2_03_difficult | 1.702 | 0.276 | 0.153 |

**Notes:**
- **Mono VO** (`pinhole.yaml`): Always tracks (91-98%) but has high ATE due to monocular scale drift.
- **Mono VIO** (`vio_mono.yaml`): Uses Ceres backend with IMU. Tracks 10/11 sequences. V2_01 fails to initialize in offline mode (marked --). Accuracy varies by sequence; the backend's scale convergence takes a few frames.
- **Stereo VIO** (`vio_stereo.yaml`): Tracks all 11 sequences at 100% with sub-20cm accuracy. **Recommended for metric accuracy.**
- Loop closing is disabled in these benchmarks (requires a DBoW2 vocabulary file not included in this repository).

This builds and runs 12 unit tests covering the Ceres backend (manifold parameterizations, error terms, marginalization, IMU errors, map operations) and the core SVO frame pipeline.

| Test | Module | What it tests |
|---|---|---|
| `test_backend_id` | svo_ceres_backend | Backend ID generation |
| `test_estimator` | svo_ceres_backend | Full estimator pipeline |
| `test_homogeneous_point_error` | svo_ceres_backend | Homogeneous point error term |
| `test_map` | svo_ceres_backend | Map data structure |
| `test_marginalization` | svo_ceres_backend | Schur complement marginalization |
| `test_pose_error` | svo_ceres_backend | Pose error term |
| `test_pose_local_parameterization` | svo_ceres_backend | Manifold Plus/Minus round-trip |
| `test_reprojection_error` | svo_ceres_backend | Reprojection error term |
| `test_relative_pose_error` | svo_ceres_backend | Relative pose error term |
| `test_speed_and_bias_error` | svo_ceres_backend | Speed and bias error term |
| `test_imu_error` | svo_ceres_backend | IMU preintegration error |
| `test_frame` | svo | Frame creation and keypoint management |

### Building with global map (GTSAM)

The global map module requires [GTSAM](https://gtsam.org/) 4.2. GTSAM is not available via Homebrew and is incompatible with Eigen 5, so it must be built from source with its bundled Eigen 3:

```sh
git clone -b 4.2 --depth 1 https://github.com/borglab/gtsam.git
cd gtsam && cmake -P /path/to/rpg_svo_pro_open/cmake/PatchGtsamBoost.cmake
cd .. && cmake -S gtsam -B gtsam/build \
  -DGTSAM_USE_SYSTEM_EIGEN=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_WITH_TBB=OFF \
  -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build gtsam/build -j$(nproc)
sudo cmake --install gtsam/build
```

> **Note:** The `PatchGtsamBoost.cmake` script removes the `boost_system` compiled library requirement, which became header-only in Boost >= 1.69. Run it from the GTSAM source root before building. If your Boost still ships `libboost_system`, you can skip this step.

Then build SVO with the global map:

```sh
cmake .. -DCMAKE_BUILD_TYPE=Release -DSVO_BUILD_GLOBAL_MAP=ON \
  -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM
cmake --build . -j$(nproc)
```

### Loop closure vocabulary

If you enable loop closing (on by default), download the DBoW2 vocabulary files:

```sh
cd svo_online_loopclosing/vocabularies && ./download_voc.sh
```

This downloads vocabulary files from the RPG server. Without these, the pipeline will crash when initializing loop closure.

### Using SVO as a library

After building, SVO can be installed and consumed by other CMake projects:

```sh
cmake --install . --prefix /usr/local
```

Then in your project's `CMakeLists.txt`:

```cmake
find_package(svo REQUIRED)
target_link_libraries(my_app PRIVATE svo::svo)
```

All library targets are exported under the `svo::` namespace (e.g. `svo::common`, `svo::direct`, `svo::ceres_backend`, `svo::img_align`, `svo::tracker`).

### Configuration

SVO is configured via YAML files. Factory functions accept a `YAML::Node`:

```cpp
#include <svo/svo_factory.h>
#include <yaml-cpp/yaml.h>

YAML::Node config = YAML::LoadFile("svo_config.yaml");
auto camera = loadCameraBundle(config);
auto svo = svo::factory::makeMono(config, camera);
```

Two main types of YAML files are needed:

1. **Parameter file** (e.g. `pinhole.yaml`, `vio_mono.yaml`) -- pipeline settings, feature detection thresholds, backend options, loop closure parameters
2. **Calibration file** (e.g. `euroc_mono.yaml`) -- camera intrinsics, distortion, IMU noise model, camera-IMU extrinsics

Example parameter and calibration files are provided under `svo_ros/param/` and `svo_ros/param/calib/`. Parameter names are unchanged from the upstream ROS version; only the loading mechanism has changed from `ros::NodeHandle` to `YAML::Node`.

See the detailed documentation below for parameter descriptions and tuning guidance.

## Documentation

* **Getting started**
    * [The visual front-end](./doc/frontend/visual_frontend.md) -- camera models, parameter files, calibration
    * [Stereo + IMU configuration](./doc/frontend/frontend_fla.md) -- calibration format, IMU parameters, resolution tuning
* **Pipeline modes**
    * [Visual-inertial odometry](./doc/vio.md) -- VIO backend, loop closure
    * [VIO + global map](./doc/global_map.md) -- iSAM2-based global bundle adjustment
* **Reference**
    * [Camera and sensor calibration](./doc/calibration.md) -- pinhole, equidistant, omnidirectional models
    * [Known issues and possible improvements](./doc/known_issues_and_improvements.md)

## What changed from upstream

This fork makes the following changes from [uzh-rpg/rpg_svo_pro_open](https://github.com/uzh-rpg/rpg_svo_pro_open):

**Build system** (Catkin/ROS to pure CMake):
* Top-level `CMakeLists.txt` with modern CMake targets and `FetchContent`
* Per-package `CMakeLists.txt` files replacing `catkin_simple` macros
* `cmake/SvoCompilerFlags.cmake` INTERFACE library for shared compiler flags
* Vendored `third_party/{minkindr, fast_neon, eigen_checks}`
* Install/export rules with `svo::` namespace aliases

**Ceres 2.2+ Manifold API migration** (replacing deprecated `LocalParameterization`):
* `PoseLocalParameterization` and `HomogeneousPointLocalParameterization` inherit `ceres::Manifold`
* `AmbientSize()`/`TangentSize()` replacing `GlobalSize()`/`LocalSize()`
* `Plus()`/`Minus()`/`PlusJacobian()`/`MinusJacobian()` API
* `SetManifold()` replacing `SetParameterization()` throughout
* `ceres::EigenQuaternionManifold` replacing `ceres::EigenQuaternionParameterization`

**ROS removal**:
* Removed `SVO_USE_ROS` compile flag and all ROS includes
* YAML-based `params_helper.h` replacing ROS parameter server reads
* Removed `CeresBackendPublisher` (pure ROS visualization)
* Factory functions extracted from `svo_ros` and adapted to `YAML::Node`
* Test data paths via compile definitions and environment variables

**C++17 compatibility fixes**:
* `std::random_shuffle` replaced with `std::shuffle`
* `std::bind2nd` replaced with lambdas
* Added missing `#include <cassert>` where needed
* Eigen 5 compatibility fixes for `ConstantReturnType`/`IdentityReturnType`

**Dropped packages** (ROS-only, not included in the build):
`svo_ros`, `svo_msgs`, `vikit_ros`, `vikit_py`, `rqt_svo`, `svo_benchmarking`, `svo_cmake`

## Troubleshooting

1. **Eigen version mismatch**: If you have multiple Eigen versions installed, ensure CMake finds the correct one by setting `-DEIGEN3_INCLUDE_DIR=/path/to/eigen3` at configure time.

2. **Ceres version too old**: This build requires Ceres 2.2+. Check with `pkg-config --modversion ceres` or look for `ceres::Manifold` in your Ceres headers. If your system Ceres is too old, [build from source](http://ceres-solver.org/installation.html).

3. **opengv build failures with Eigen 5**: If you have Eigen 5 installed, opengv's bundled `FindEigen.cmake` may fail. The top-level `CMakeLists.txt` includes workarounds for this, but if issues persist, set `EIGEN_INCLUDE_DIR` and `EIGEN_VERSION` cache variables explicitly.

4. **Pipeline crashes with loop closure enabled**: If the pipeline crashes calling `svo::loadVoc()`, make sure to download the vocabulary files:

    ```sh
    cd svo_online_loopclosing/vocabularies && ./download_voc.sh
    ```

5. **CMake 4.x errors with FetchContent dependencies**: The build uses `cmake_minimum_required(VERSION 3.16...4.1)` and sets `CMAKE_POLICY_VERSION_MINIMUM` to handle old `cmake_minimum_required` in fetched dependencies (opengv, DBoW2). If you see policy errors, ensure you are using CMake 3.16 or newer.

6. **`-msse2` error on Apple Silicon**: The build detects ARM vs x86 via `CMAKE_SYSTEM_PROCESSOR` and only applies `-msse2` on x86. If you see this error, ensure you are not cross-compiling or overriding the processor detection.

## Acknowledgement

Thanks to Simon Klenk, Manasi Muglikar, Giovanni Cioffi and Javier Hidalgo-Carrio for their valuable help and comments for the open source code.

The work is made possible thanks to the efforts of many contributors from RPG. Apart from the authors listed in the above papers, Titus Cieslewski and Henri Rebecq made significant contributions to the visual front-end. Jeffrey Delmerico made great efforts to apply SVO on different real robots, which in turn helped improve the pipeline. Many PhD and master students and lab engineers have also contributed to the code.

The Ceres-based optimization back-end is based on code developed at [Zurich-eye](https://www.wysszurich.uzh.ch/projects/completed-projects/zurich-eye), a spin-off from RPG. Jonathan Huber is the main contributor that integrated the back-end with SVO. Kunal Shrivastava (now CEO of [SUIND](https://suind.com/)) developed the loop closure module during his semester project and internship at RPG. The integration of the iSAM2-based global map was developed by Zichao Zhang.

We would like to thank our collaborators at [Prophesee](https://www.prophesee.ai/) for pointing out several bugs in the visual front-end. Part of the code was developed during a funded project with [Huawei](https://www.huawei.com/en/).
