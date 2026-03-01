// EuRoC Stereo Visual-Inertial Odometry example.
//
// Usage:
//   euroc_stereo_vio --dataset_dir /path/to/MH_01_easy/mav0
//
// Reads stereo images from cam0/ + cam1/ and IMU data from imu0/ in the
// EuRoC ASL format, runs the SVO stereo VIO pipeline, and writes a
// TUM-format trajectory.

#include <fstream>
#include <iomanip>
#include <string>
#include <unordered_map>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <svo/svo_factory.h>
#include <svo/frame_handler_stereo.h>
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
DEFINE_string(config, "", "Parameter YAML file (default: bundled vio_stereo.yaml)");
DEFINE_string(calib, "", "Calibration YAML file (default: bundled euroc_stereo.yaml)");
DEFINE_string(output, "traj_estimate.txt", "Output trajectory file (TUM format)");
DEFINE_double(start_offset_sec, 0.0, "Skip this many seconds from the start");

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  CHECK(!FLAGS_dataset_dir.empty()) << "Please specify --dataset_dir";

  const std::string src_dir = std::string(__FILE__).substr(
      0, std::string(__FILE__).rfind('/'));
  const std::string param_file = FLAGS_config.empty()
      ? src_dir + "/../svo_ros/param/vio_stereo.yaml"
      : FLAGS_config;
  const std::string calib_file = FLAGS_calib.empty()
      ? src_dir + "/../svo_ros/param/calib/euroc_stereo.yaml"
      : FLAGS_calib;

  // 1. Load and merge config
  YAML::Node config = svo::euroc::mergeConfig(param_file, calib_file);

  // Disable loop closing by default (requires vocabulary file not in repo)
  if (!config["runlc"] || config["runlc"].as<bool>())
  {
    config["runlc"] = false;
    LOG(INFO) << "Loop closing disabled (vocabulary not available)";
  }

  // 2. Create stereo pipeline
  auto svo = svo::factory::makeStereo(config);
  CHECK(svo) << "Failed to create stereo pipeline";

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

  // 7. Load image lists for both cameras
  const std::string cam0_dir = FLAGS_dataset_dir + "/cam0";
  const std::string cam1_dir = FLAGS_dataset_dir + "/cam1";
  auto images_cam0 = svo::euroc::loadEurocImages(cam0_dir);
  auto images_cam1 = svo::euroc::loadEurocImages(cam1_dir);
  CHECK(!images_cam0.empty()) << "No images loaded from " << cam0_dir;
  CHECK(!images_cam1.empty()) << "No images loaded from " << cam1_dir;

  // Build cam1 timestamp lookup for matching stereo pairs
  std::unordered_map<int64_t, std::string> cam1_map;
  for (const auto& [ts, path] : images_cam1)
    cam1_map[ts] = path;
  LOG(INFO) << "cam0: " << images_cam0.size() << " images, cam1: "
            << images_cam1.size() << " images";

  const int64_t start_ts = images_cam0.front().first
      + static_cast<int64_t>(FLAGS_start_offset_sec * 1e9);

  // 8. Open output file
  std::ofstream traj_ofs(FLAGS_output);
  CHECK(traj_ofs.is_open()) << "Cannot open " << FLAGS_output;
  traj_ofs << "# TUM trajectory format: timestamp tx ty tz qx qy qz qw\n";

  // 9. Start pipeline
  svo->start();
  LOG(INFO) << "Processing stereo pairs...";

  size_t n_processed = 0;
  size_t n_tracked = 0;
  double total_time = 0.0;

  // 10. Main loop — iterate cam0 and find matching cam1 by timestamp
  for (size_t i = 0; i < images_cam0.size(); ++i)
  {
    const int64_t ts_ns = images_cam0[i].first;
    if (ts_ns < start_ts)
      continue;

    auto it = cam1_map.find(ts_ns);
    if (it == cam1_map.end())
      continue;  // no matching cam1 frame

    cv::Mat img0 = cv::imread(images_cam0[i].second, cv::IMREAD_GRAYSCALE);
    cv::Mat img1 = cv::imread(it->second, cv::IMREAD_GRAYSCALE);
    if (img0.empty() || img1.empty())
    {
      LOG(WARNING) << "Failed to read stereo pair at timestamp " << ts_ns;
      continue;
    }

    if (!svo::euroc::setImuPrior(*svo, imu_handler, ts_ns))
      continue;

    svo->addImageBundle({img0, img1}, static_cast<uint64_t>(ts_ns));
    n_processed++;
    total_time += svo->lastProcessingTime();

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

    if (n_processed % 100 == 0)
    {
      LOG(INFO) << "Frame " << n_processed << "/" << images_cam0.size()
                << " | Stage: " << static_cast<int>(svo->stage())
                << " | Obs: " << svo->lastNumObservations();
    }
  }

  traj_ofs.close();

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
