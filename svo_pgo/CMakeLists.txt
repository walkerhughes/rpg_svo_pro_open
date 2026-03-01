add_library(svo_pgo src/pgo.cpp)
add_library(svo::pgo ALIAS svo_pgo)
target_include_directories(svo_pgo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(svo_pgo
  PUBLIC Eigen3::Eigen Ceres::ceres svo_common
  PRIVATE svo_compiler_flags glog::glog
)
