// Shared utilities for EuRoC ASL dataset examples.
#pragma once

#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>

#include <svo/common/conversions.h>
#include <svo/common/transformation.h>
#include <svo/frame_handler_base.h>
#include <svo/imu_handler.h>

namespace svo {
namespace euroc {

/// Parse an EuRoC cam<N>/data.csv and return (timestamp_ns, filepath) pairs.
inline std::vector<std::pair<int64_t, std::string>>
loadEurocImages(const std::string& cam_dir)
{
  const std::string csv_path = cam_dir + "/data.csv";
  std::ifstream fs(csv_path);
  CHECK(fs.is_open()) << "Cannot open " << csv_path;

  std::vector<std::pair<int64_t, std::string>> images;
  std::string line;
  // Skip header line
  std::getline(fs, line);
  while (std::getline(fs, line))
  {
    if (line.empty() || line[0] == '#')
      continue;
    // Format: timestamp [ns], filename
    auto comma = line.find(',');
    if (comma == std::string::npos)
      continue;
    int64_t stamp = std::stoll(line.substr(0, comma));
    std::string filename = line.substr(comma + 1);
    // Trim whitespace
    while (!filename.empty() && (filename.front() == ' ' || filename.front() == '\t'))
      filename.erase(filename.begin());
    while (!filename.empty() && (filename.back() == ' ' || filename.back() == '\r' || filename.back() == '\n'))
      filename.pop_back();
    images.emplace_back(stamp, cam_dir + "/data/" + filename);
  }
  LOG(INFO) << "Loaded " << images.size() << " image entries from " << csv_path;
  return images;
}

/// Load IMU measurements from EuRoC imu0/data.csv using ImuHandler's native parser.
inline void loadEurocImu(const ImuHandler::Ptr& imu_handler,
                         const std::string& imu_csv_path)
{
  CHECK(imu_handler->loadImuMeasurementsFromCsvFile(imu_csv_path))
      << "Failed to load IMU measurements from " << imu_csv_path;
  LOG(INFO) << "Loaded IMU measurements from " << imu_csv_path;
}

/// Replicate SvoInterface::setImuPrior(): gravity alignment on first frame,
/// incremental rotation prior thereafter.
inline bool setImuPrior(FrameHandlerBase& svo,
                        const ImuHandler::Ptr& imu_handler,
                        int64_t timestamp_ns)
{
  const double ts_sec = static_cast<double>(timestamp_ns)
                        * common::conversions::kNanoSecondsToSeconds;

  if (svo.getBundleAdjuster())
  {
    // Backend handles priors; just wait for enough IMU measurements at startup.
    if (!svo.hasStarted())
    {
      if (imu_handler->getMeasurementsCopy().size() < 10u)
        return false;
    }
    return true;
  }

  if (imu_handler && !svo.hasStarted())
  {
    // Set initial orientation from gravity.
    Quaternion R_imu_world;
    if (imu_handler->getInitialAttitude(ts_sec, R_imu_world))
    {
      svo.setRotationPrior(R_imu_world);
    }
    else
    {
      return false;
    }
  }
  else if (imu_handler && svo.getLastFrames())
  {
    // Set incremental rotation prior.
    const double last_ts_sec =
        svo.getLastFrames()->getMinTimestampNanoseconds()
        * common::conversions::kNanoSecondsToSeconds;
    Quaternion R_lastimu_newimu;
    if (imu_handler->getRelativeRotationPrior(
            last_ts_sec, ts_sec, false, R_lastimu_newimu))
    {
      svo.setRotationIncrementPrior(R_lastimu_newimu);
    }
  }
  return true;
}

/// Write a single TUM-format trajectory line:
/// timestamp_sec tx ty tz qx qy qz qw
inline void saveTumTrajectory(std::ofstream& ofs,
                              double timestamp_sec,
                              const Transformation& T_W_B)
{
  const Eigen::Vector3d& p = T_W_B.getPosition();
  const Eigen::Quaterniond& q = T_W_B.getRotation().toImplementation();
  ofs << std::fixed << std::setprecision(9) << timestamp_sec << " "
      << std::setprecision(6)
      << p.x() << " " << p.y() << " " << p.z() << " "
      << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
      << "\n";
}

/// Load a parameter YAML and inject the calib_file path so the factory
/// functions can find the calibration.
inline YAML::Node mergeConfig(const std::string& param_yaml,
                              const std::string& calib_yaml)
{
  YAML::Node config = YAML::LoadFile(param_yaml);
  config["calib_file"] = calib_yaml;
  return config;
}

}  // namespace euroc
}  // namespace svo
