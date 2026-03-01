add_library(svo_ceres_backend
  src/ceres_backend_interface.cpp
  src/ceres_backend_factory.cpp
  src/error_interface.cpp
  src/estimator.cpp
  src/homogeneous_point_error.cpp
  src/homogeneous_point_local_parameterization.cpp
  src/homogeneous_point_parameter_block.cpp
  src/imu_error.cpp
  src/local_parameterization_additional_interfaces.cpp
  src/map.cpp
  src/map_alignment.cpp
  src/marginalization_error.cpp
  src/motion_detector.cpp
  src/outlier_rejection.cpp
  src/pose_error.cpp
  src/pose_local_parameterization.cpp
  src/pose_parameter_block.cpp
  src/relative_pose_error.cpp
  src/speed_and_bias_error.cpp
  src/speed_and_bias_parameter_block.cpp
)
add_library(svo::ceres_backend ALIAS svo_ceres_backend)
target_include_directories(svo_ceres_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/svo/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(svo_ceres_backend
  PUBLIC Eigen3::Eigen Ceres::ceres svo_common svo_vio_common $<BUILD_INTERFACE:opengv>
  PRIVATE svo_compiler_flags ${OpenCV_LIBS} glog::glog yaml-cpp
)

if(SVO_BUILD_TESTS)
  add_executable(test_backend_id test/test_backend_id.cpp)
  target_link_libraries(test_backend_id PRIVATE svo_ceres_backend GTest::GTest GTest::Main)
  add_test(NAME test_backend_id COMMAND test_backend_id)

  add_executable(test_estimator test/test_estimator.cpp)
  target_link_libraries(test_estimator PRIVATE svo_ceres_backend GTest::GTest GTest::Main)
  add_test(NAME test_estimator COMMAND test_estimator)

  add_executable(test_homogeneous_point_error test/test_homogeneous_point_error.cpp)
  target_link_libraries(test_homogeneous_point_error PRIVATE svo_ceres_backend GTest::GTest GTest::Main)
  add_test(NAME test_homogeneous_point_error COMMAND test_homogeneous_point_error)

  add_executable(test_map test/test_map.cpp)
  target_link_libraries(test_map PRIVATE svo_ceres_backend GTest::GTest GTest::Main)
  add_test(NAME test_map COMMAND test_map)

  add_executable(test_marginalization test/test_marginalization.cpp)
  target_link_libraries(test_marginalization PRIVATE svo_ceres_backend GTest::GTest GTest::Main)
  add_test(NAME test_marginalization COMMAND test_marginalization)

  add_executable(test_pose_error test/test_pose_error.cpp)
  target_link_libraries(test_pose_error PRIVATE svo_ceres_backend GTest::GTest GTest::Main)
  add_test(NAME test_pose_error COMMAND test_pose_error)

  add_executable(test_pose_local_parameterization test/test_pose_local_parameterization.cpp)
  target_link_libraries(test_pose_local_parameterization PRIVATE svo_ceres_backend eigen_checks GTest::GTest GTest::Main)
  add_test(NAME test_pose_local_parameterization COMMAND test_pose_local_parameterization)

  add_executable(test_reprojection_error test/test_reprojection_error.cpp)
  target_link_libraries(test_reprojection_error PRIVATE svo_ceres_backend GTest::GTest GTest::Main)
  add_test(NAME test_reprojection_error COMMAND test_reprojection_error)

  add_executable(test_relative_pose_error test/test_relative_pose_error.cpp)
  target_link_libraries(test_relative_pose_error PRIVATE svo_ceres_backend GTest::GTest GTest::Main)
  add_test(NAME test_relative_pose_error COMMAND test_relative_pose_error)

  add_executable(test_speed_and_bias_error test/test_speed_and_bias_error.cpp)
  target_link_libraries(test_speed_and_bias_error PRIVATE svo_ceres_backend GTest::GTest GTest::Main)
  add_test(NAME test_speed_and_bias_error COMMAND test_speed_and_bias_error)

  add_executable(test_imu_error test/test_imu_error.cpp)
  target_link_libraries(test_imu_error PRIVATE svo_ceres_backend GTest::GTest GTest::Main)
  add_test(NAME test_imu_error COMMAND test_imu_error)
endif()
