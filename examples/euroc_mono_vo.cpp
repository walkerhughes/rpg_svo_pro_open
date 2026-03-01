// EuRoC Monocular Visual Odometry example (no IMU, no backend).
//
// Usage:
//   euroc_mono_vo --dataset_dir /path/to/MH_01_easy/mav0
//
// The simplest example: reads monocular images from cam0/ and runs the SVO
// visual front-end only. Automatically restarts on tracking failure.

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

#include "euroc_common.h"

DEFINE_string(dataset_dir, "", "Path to EuRoC mav0/ directory (required)");
DEFINE_string(config, "", "Parameter YAML file (default: bundled pinhole.yaml)");
DEFINE_string(calib, "", "Calibration YAML file (default: bundled euroc_mono.yaml)");
DEFINE_string(output, "traj_estimate.txt", "Output trajectory file (TUM format)");

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  CHECK(!FLAGS_dataset_dir.empty()) << "Please specify --dataset_dir";

  const std::string src_dir = std::string(__FILE__).substr(
      0, std::string(__FILE__).rfind('/'));
  const std::string param_file = FLAGS_config.empty()
      ? src_dir + "/../svo_ros/param/pinhole.yaml"
      : FLAGS_config;
  const std::string calib_file = FLAGS_calib.empty()
      ? src_dir + "/../svo_ros/param/calib/euroc_mono.yaml"
      : FLAGS_calib;

  // 1. Load and merge config
  YAML::Node config = svo::euroc::mergeConfig(param_file, calib_file);

  // 2. Create pipeline (no IMU, no backend)
  auto svo = svo::factory::makeMono(config);
  CHECK(svo) << "Failed to create mono pipeline";

  // 3. Load image list
  const std::string cam_dir = FLAGS_dataset_dir + "/cam0";
  auto images = svo::euroc::loadEurocImages(cam_dir);
  CHECK(!images.empty()) << "No images loaded from " << cam_dir;

  // 4. Open output file
  std::ofstream traj_ofs(FLAGS_output);
  CHECK(traj_ofs.is_open()) << "Cannot open " << FLAGS_output;
  traj_ofs << "# TUM trajectory format: timestamp tx ty tz qx qy qz qw\n";

  // 5. Start pipeline
  svo->start();
  LOG(INFO) << "Processing " << images.size() << " images (pure VO, no IMU)...";

  size_t n_processed = 0;
  size_t n_tracked = 0;
  double total_time = 0.0;

  // 6. Main loop
  for (const auto& [ts_ns, img_path] : images)
  {
    cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
    if (img.empty())
    {
      LOG(WARNING) << "Failed to read " << img_path;
      continue;
    }

    svo->addImageBundle({img}, static_cast<uint64_t>(ts_ns));
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
    else if (svo->stage() == svo::Stage::kRelocalization)
    {
      // Auto-restart on tracking failure
      LOG(WARNING) << "Tracking lost at frame " << n_processed << ", restarting";
      svo->start();
    }

    if (n_processed % 100 == 0)
    {
      LOG(INFO) << "Frame " << n_processed << "/" << images.size()
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
