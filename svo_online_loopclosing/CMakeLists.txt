if(NOT SVO_BUILD_LOOP_CLOSING)
  return()
endif()

add_library(svo_online_loopclosing
  src/bow.cpp
  src/geometric_verification.cpp
  src/loop_closing.cpp
  src/map_alignment.cpp
  src/read_file.cpp
)
add_library(svo::online_loopclosing ALIAS svo_online_loopclosing)
target_include_directories(svo_online_loopclosing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(svo_online_loopclosing
  PUBLIC Eigen3::Eigen Ceres::ceres svo_common svo_ceres_backend
  PRIVATE svo_compiler_flags ${OpenCV_LIBS} glog::glog $<BUILD_INTERFACE:DBoW2>
)
