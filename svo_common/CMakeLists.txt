file(GLOB SVO_COMMON_SRC src/*.cpp)
add_library(svo_common ${SVO_COMMON_SRC})
add_library(svo::common ALIAS svo_common)
target_include_directories(svo_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(svo_common
  PUBLIC Eigen3::Eigen ${OpenCV_LIBS} vikit_cameras vikit_common minkindr svo_vio_common fast
  PRIVATE svo_compiler_flags glog::glog
)
