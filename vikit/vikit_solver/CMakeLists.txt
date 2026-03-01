file(GLOB VIKIT_SOLVER_SRC src/*.cpp)
add_library(vikit_solver ${VIKIT_SOLVER_SRC})
add_library(svo::vikit_solver ALIAS vikit_solver)
target_include_directories(vikit_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vikit_solver PUBLIC Eigen3::Eigen PRIVATE svo_compiler_flags glog::glog)
