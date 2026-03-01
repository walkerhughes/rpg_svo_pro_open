add_library(svo_test_utils
  src/test_utils.cpp
  src/imu_simulation.cpp
  src/synthetic_dataset.cpp
  src/simple_raytracer.cpp
  src/trajectory_generator.cpp
  src/point_cloud_generator.cpp
)
add_library(svo::test_utils ALIAS svo_test_utils)
target_include_directories(svo_test_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(svo_test_utils
  PUBLIC Eigen3::Eigen ${OpenCV_LIBRARIES} svo_common svo_vio_common vikit_cameras
  PRIVATE svo_compiler_flags glog::glog
)
target_compile_definitions(svo_test_utils PUBLIC
  SVO_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/svo/test/data"
)
