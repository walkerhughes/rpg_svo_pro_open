if(NOT SVO_BUILD_GLOBAL_MAP)
  return()
endif()

find_package(GTSAM REQUIRED)

add_library(svo_global_map
  src/gtsam/gtsam_optimizer.cpp
  src/gtsam/graph_manager.cpp
  src/global_map.cpp
  src/null_stream.cpp
)
add_library(svo::global_map ALIAS svo_global_map)
target_include_directories(svo_global_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(svo_global_map
  PUBLIC Eigen3::Eigen svo_common svo_ceres_backend vikit_cameras gtsam
  PRIVATE svo_compiler_flags glog::glog
)
