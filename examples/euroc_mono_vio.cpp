// EuRoC Monocular Visual-Inertial Odometry example.
//
// Usage:
//   euroc_mono_vio --dataset_dir /path/to/MH_01_easy/mav0
//
// Reads images from cam0/ and IMU data from imu0/ in the EuRoC ASL format,
// runs the SVO monocular VIO pipeline, and writes a TUM-format trajectory.

#include <fstream>
#include <iomanip>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <svo/svo_factory.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_base.h>
#include <svo/common/frame.h>
#include <svo/imu_handler.h>
#include <svo/ceres_backend_factory.h>
#include <svo/ceres_backend_interface.hpp>

#ifdef SVO_LOOP_CLOSING
#include <svo/online_loopclosing/loop_closing.h>
#endif

#include "euroc_common.h"

DEFINE_string(dataset_dir, "", "Path to EuRoC mav0/ directory (required)");
DEFINE_string(config, "", "Parameter YAML file (default: bundled vio_mono.yaml)");
DEFINE_string(calib, "", "Calibration YAML file (default: bundled euroc_mono.yaml)");
DEFINE_string(output, "traj_estimate.txt", "Output trajectory file (TUM format)");
DEFINE_double(start_offset_sec, 0.0, "Skip this many seconds from the start");

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  CHECK(!FLAGS_dataset_dir.empty()) << "Please specify --dataset_dir";

  // Resolve default config paths relative to the source tree.
  const std::string src_dir = std::string(__FILE__).substr(
      0, std::string(__FILE__).rfind('/'));
  const std::string param_file = FLAGS_config.empty()
      ? src_dir + "/../svo_ros/param/vio_mono.yaml"
      : FLAGS_config;
  const std::string calib_file = FLAGS_calib.empty()
      ? src_dir + "/../svo_ros/param/calib/euroc_mono.yaml"
      : FLAGS_calib;

  // 1. Load and merge config
  YAML::Node config = svo::euroc::mergeConfig(param_file, calib_file);

  // Disable loop closing by default (requires vocabulary file not in repo)
  if (!config["runlc"] || config["runlc"].as<bool>())
  {
    config["runlc"] = false;
    LOG(INFO) << "Loop closing disabled (vocabulary not available)";
  }

  // 2. Create pipeline
  auto svo = svo::factory::makeMono(config);
  CHECK(svo) << "Failed to create mono pipeline";

  // 3. Create IMU handler and attach
  auto imu_handler = svo::factory::getImuHandler(config);
  CHECK(imu_handler) << "Failed to create IMU handler";
  svo->imu_handler_ = imu_handler;

  // 4. Create Ceres backend and attach
  auto ncam = svo->getNCamera();
  auto backend = svo::ceres_backend_factory::makeBackend(config, ncam);
  CHECK(backend) << "Failed to create backend";
  svo->setBundleAdjuster(backend);
  backend->setImu(imu_handler);

  // 5. Optionally attach loop closing
#ifdef SVO_LOOP_CLOSING
  if (vk::param<bool>(config, "runlc", false))
  {
    auto lc = svo::factory::getLoopClosingModule(config, ncam);
    svo->lc_ = std::move(lc);
    LOG(INFO) << "Loop closing enabled";
  }
#endif

  // 6. Load IMU data
  const std::string imu_csv = FLAGS_dataset_dir + "/imu0/data.csv";
  svo::euroc::loadEurocImu(imu_handler, imu_csv);

  // 7. Load image list
  const std::string cam_dir = FLAGS_dataset_dir + "/cam0";
  auto images = svo::euroc::loadEurocImages(cam_dir);
  CHECK(!images.empty()) << "No images loaded from " << cam_dir;

  // Determine start timestamp
  const int64_t start_ts = images.front().first
      + static_cast<int64_t>(FLAGS_start_offset_sec * 1e9);

  // 8. Open output file
  std::ofstream traj_ofs(FLAGS_output);
  CHECK(traj_ofs.is_open()) << "Cannot open " << FLAGS_output;
  traj_ofs << "# TUM trajectory format: timestamp tx ty tz qx qy qz qw\n";

  // 9. Start pipeline
  svo->start();
  LOG(INFO) << "Processing " << images.size() << " images...";

  size_t n_processed = 0;
  size_t n_tracked = 0;
  double total_time = 0.0;

  // 10. Main loop
  for (const auto& [ts_ns, img_path] : images)
  {
    if (ts_ns < start_ts)
      continue;

    cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
    if (img.empty())
    {
      LOG(WARNING) << "Failed to read " << img_path;
      continue;
    }

    // Set IMU prior
    if (!svo::euroc::setImuPrior(*svo, imu_handler, ts_ns))
      continue;

    // Process frame
    svo->addImageBundle({img}, static_cast<uint64_t>(ts_ns));
    n_processed++;
    total_time += svo->lastProcessingTime();

    // Write trajectory if tracking
    if (svo->stage() == svo::Stage::kTracking)
    {
      n_tracked++;
      auto last_frames = svo->getLastFrames();
      if (last_frames)
      {
        svo::Transformation T_W_B = last_frames->get_T_W_B();
        double ts_sec = static_cast<double>(ts_ns) * 1e-9;
        svo::euroc::saveTumTrajectory(traj_ofs, ts_sec, T_W_B);
      }
    }

    // Print progress every 100 frames
    if (n_processed % 100 == 0)
    {
      LOG(INFO) << "Frame " << n_processed << "/" << images.size()
                << " | Stage: " << static_cast<int>(svo->stage())
                << " | Obs: " << svo->lastNumObservations();
    }
  }

  traj_ofs.close();

  // 11. Summary
  LOG(INFO) << "=== Summary ===";
  LOG(INFO) << "Frames processed: " << n_processed;
  LOG(INFO) << "Frames tracked:   " << n_tracked
            << " (" << (n_processed > 0 ? 100.0 * n_tracked / n_processed : 0)
            << "%)";
  LOG(INFO) << "Avg processing time: "
            << (n_processed > 0 ? 1000.0 * total_time / n_processed : 0)
            << " ms";
  LOG(INFO) << "Trajectory saved to: " << FLAGS_output;

  return 0;
}
