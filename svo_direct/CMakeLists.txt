file(GLOB SVO_DIRECT_SRC src/*.cpp)
add_library(svo_direct ${SVO_DIRECT_SRC})
add_library(svo::direct ALIAS svo_direct)
target_include_directories(svo_direct PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(svo_direct
  PUBLIC Eigen3::Eigen svo_common svo_vio_common
  PRIVATE svo_compiler_flags vikit_solver rpg_common fast ${OpenCV_LIBS} glog::glog
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(svo_direct PRIVATE OpenMP::OpenMP_CXX)
endif()
