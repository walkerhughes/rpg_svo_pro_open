add_library(rpg_common INTERFACE)
add_library(svo::rpg_common ALIAS rpg_common)
target_include_directories(rpg_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rpg_common INTERFACE Eigen3::Eigen minkindr)
