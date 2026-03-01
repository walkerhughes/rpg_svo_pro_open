file(GLOB VIKIT_CAMERAS_SRC src/*.cpp)
add_library(vikit_cameras ${VIKIT_CAMERAS_SRC})
add_library(svo::vikit_cameras ALIAS vikit_cameras)
target_include_directories(vikit_cameras PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vikit_cameras
  PUBLIC Eigen3::Eigen ${OpenCV_LIBS} vikit_common yaml-cpp
  PRIVATE svo_compiler_flags glog::glog
)
