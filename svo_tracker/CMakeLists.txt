file(GLOB SVO_TRACKER_SRC src/*.cpp)
add_library(svo_tracker ${SVO_TRACKER_SRC})
add_library(svo::tracker ALIAS svo_tracker)
target_include_directories(svo_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(svo_tracker
  PUBLIC Eigen3::Eigen svo_common svo_direct
  PRIVATE svo_compiler_flags ${OpenCV_LIBS} glog::glog
)
