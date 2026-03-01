add_library(svo_img_align
  src/sparse_img_align_base.cpp
  src/sparse_img_align.cpp
)
add_library(svo::img_align ALIAS svo_img_align)
target_include_directories(svo_img_align PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(svo_img_align
  PUBLIC Eigen3::Eigen svo_common svo_vio_common svo_direct
  PRIVATE svo_compiler_flags vikit_solver ${OpenCV_LIBS} glog::glog
)
