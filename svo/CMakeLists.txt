add_library(svo
  src/frame_handler_array.cpp
  src/frame_handler_base.cpp
  src/frame_handler_mono.cpp
  src/frame_handler_stereo.cpp
  src/imu_handler.cpp
  src/initialization.cpp
  src/io.cpp
  src/map.cpp
  src/pose_optimizer.cpp
  src/reprojector.cpp
  src/stereo_triangulation.cpp
  src/svo_factory.cpp
)
add_library(svo::svo ALIAS svo)
target_include_directories(svo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(svo
  PUBLIC Eigen3::Eigen svo_common svo_direct svo_img_align svo_tracker svo_vio_common svo_ceres_backend rpg_common vikit_solver vikit_cameras svo_compiler_flags
  PRIVATE ${OpenCV_LIBS} glog::glog yaml-cpp
)
if(SVO_BUILD_LOOP_CLOSING)
  target_link_libraries(svo PUBLIC svo_online_loopclosing svo_pgo PRIVATE $<BUILD_INTERFACE:DBoW2>)
endif()
if(SVO_BUILD_GLOBAL_MAP)
  target_link_libraries(svo PUBLIC svo_global_map)
endif()

if(SVO_BUILD_TESTS)
  add_executable(test_frame test/test_frame.cpp)
  target_link_libraries(test_frame PRIVATE svo svo_test_utils eigen_checks GTest::GTest GTest::Main)
  target_compile_definitions(test_frame PRIVATE SVO_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/svo/test/data")
  add_test(NAME test_frame COMMAND test_frame)
endif()
