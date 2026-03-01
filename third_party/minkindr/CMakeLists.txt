add_library(minkindr INTERFACE)
add_library(svo::minkindr ALIAS minkindr)
target_include_directories(minkindr INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(minkindr INTERFACE Eigen3::Eigen)
