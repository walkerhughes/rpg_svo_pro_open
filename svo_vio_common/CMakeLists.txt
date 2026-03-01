file(GLOB SVO_VIO_COMMON_SRC src/*.cpp)
add_library(svo_vio_common ${SVO_VIO_COMMON_SRC})
add_library(svo::vio_common ALIAS svo_vio_common)
target_include_directories(svo_vio_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/svo_common/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(svo_vio_common
  PUBLIC Eigen3::Eigen minkindr vikit_common vikit_cameras
  PRIVATE svo_compiler_flags glog::glog
)
