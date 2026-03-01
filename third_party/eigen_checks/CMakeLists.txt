add_library(eigen_checks INTERFACE)
add_library(svo::eigen_checks ALIAS eigen_checks)
target_include_directories(eigen_checks INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
