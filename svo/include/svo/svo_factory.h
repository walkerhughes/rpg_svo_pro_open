#pragma once
#include <memory>
#include <yaml-cpp/yaml.h>
#include <svo/common/camera_fwd.h>

namespace svo {
class ImuHandler;
class LoopClosing;
class GlobalMap;
class FrameHandlerMono;
class FrameHandlerStereo;
class FrameHandlerArray;
class FrameHandlerBase;

namespace factory {
std::shared_ptr<ImuHandler> getImuHandler(const YAML::Node& config);
#ifdef SVO_LOOP_CLOSING
std::shared_ptr<LoopClosing> getLoopClosingModule(const YAML::Node& config, const CameraBundlePtr& cam = nullptr);
#endif
std::shared_ptr<FrameHandlerMono> makeMono(const YAML::Node& config, const CameraBundlePtr& cam = nullptr);
std::shared_ptr<FrameHandlerStereo> makeStereo(const YAML::Node& config, const CameraBundlePtr& cam = nullptr);
std::shared_ptr<FrameHandlerArray> makeArray(const YAML::Node& config, const CameraBundlePtr& cam = nullptr);
} // namespace factory
} // namespace svo
