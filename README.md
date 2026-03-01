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
* *Loop closure*: Dorian Gálvez-López and Juan D. Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences. TRO, 2012. [bibtex](./doc/bib/Galvez12tro.bib)

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
* Google Test (for tests)

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
| `SVO_BUILD_LOOP_CLOSING` | `ON` | Build loop closing and pose graph optimization modules |
| `SVO_BUILD_GLOBAL_MAP` | `OFF` | Build global map modules (requires GTSAM) |

Example with tests:

```sh
cmake .. -DCMAKE_BUILD_TYPE=Release -DSVO_BUILD_TESTS=ON
cmake --build . -j$(nproc)
ctest --output-on-failure
```

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

See the original [documentation](./doc/frontend/visual_frontend.md) for parameter descriptions. Parameter names are unchanged from the ROS version; only the loading mechanism has changed from `ros::NodeHandle` to `YAML::Node`.

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

**Dropped packages** (ROS-only, not included in the build):
`svo_ros`, `svo_msgs`, `vikit_ros`, `vikit_py`, `rqt_svo`, `svo_benchmarking`, `svo_cmake`

## Instructions

* Get started: running the pipeline
    * [The visual front-end](./doc/frontend/visual_frontend.md)
    * [Visual-inertial odometry](./doc/vio.md)
    * [VIO + global map](./doc/global_map.md)
* [Benchmarking](./doc/benchmarking.md)
* [Camera and sensor calibration](./doc/calibration.md)
* [Known issues and possible improvements](./doc/known_issues_and_improvements.md)

## Troubleshooting

1. **Eigen version mismatch**: If you have multiple Eigen versions installed, ensure CMake finds the correct one by setting `-DEIGEN3_INCLUDE_DIR=/path/to/eigen3` at configure time.

2. **Ceres version too old**: This build requires Ceres 2.2+. Check with `pkg-config --modversion ceres` or look for `ceres::Manifold` in your Ceres headers. If your system Ceres is too old, [build from source](http://ceres-solver.org/installation.html).

3. **opengv build failures with Eigen 5**: If you have Eigen 5 installed, opengv's bundled `FindEigen.cmake` may fail. The top-level `CMakeLists.txt` includes workarounds for this, but if issues persist, set `EIGEN_INCLUDE_DIR` and `EIGEN_VERSION` cache variables explicitly.

4. **Pipeline crashes with loop closure enabled**: If the pipeline crashes calling `svo::loadVoc()`, make sure to download the vocabulary files:

    ```sh
    cd svo_online_loopclosing/vocabularies && ./download_voc.sh
    ```

## Acknowledgement

Thanks to Simon Klenk, Manasi Muglikar, Giovanni Cioffi and Javier Hidalgo-Carrió for their valuable help and comments for the open source code.

The work is made possible thanks to the efforts of many contributors from RPG. Apart from the authors listed in the above papers, Titus Cieslewski and Henri Rebecq made significant contributions to the visual front-end. Jeffrey Delmerico made great efforts to apply SVO on different real robots, which in turn helped improve the pipeline. Many PhD and master students and lab engineers have also contributed to the code.

The Ceres-based optimization back-end is based on code developed at [Zurich-eye](https://www.wysszurich.uzh.ch/projects/completed-projects/zurich-eye), a spin-off from RPG. Jonathan Huber is the main contributor that integrated the back-end with SVO. Kunal Shrivastava (now CEO of [SUIND](https://suind.com/)) developed the loop closure module during his semester project and internship at RPG. The integration of the iSAM2-based global map was developed by Zichao Zhang.

We would like to thank our collaborators at [Prophesee](https://www.prophesee.ai/) for pointing out several bugs in the visual front-end. Part of the code was developed during a funded project with [Huawei](https://www.huawei.com/en/).
