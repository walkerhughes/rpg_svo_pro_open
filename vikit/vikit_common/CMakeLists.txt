file(GLOB VIKIT_COMMON_SRC src/*.cpp)
add_library(vikit_common ${VIKIT_COMMON_SRC})
add_library(svo::vikit_common ALIAS vikit_common)
target_include_directories(vikit_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vikit_common
  PUBLIC Eigen3::Eigen ${OpenCV_LIBS} minkindr yaml-cpp
  PRIVATE svo_compiler_flags glog::glog
)
