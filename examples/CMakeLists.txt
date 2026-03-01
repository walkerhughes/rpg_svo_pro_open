# Example programs for running SVO on the EuRoC MAV dataset.
#
# The svo library already pulls in svo_ceres_backend (and svo_online_loopclosing
# when SVO_BUILD_LOOP_CLOSING is ON) as PUBLIC dependencies, so we only need
# to link svo plus the direct dependencies used by the example sources.

set(EUROC_COMMON_LIBS
  svo
  yaml-cpp
  glog::glog
  gflags
  ${OpenCV_LIBS}
)

# --- Mono VIO ---
add_executable(euroc_mono_vio euroc_mono_vio.cpp)
target_link_libraries(euroc_mono_vio PRIVATE ${EUROC_COMMON_LIBS})

# --- Stereo VIO ---
add_executable(euroc_stereo_vio euroc_stereo_vio.cpp)
target_link_libraries(euroc_stereo_vio PRIVATE ${EUROC_COMMON_LIBS})

# --- Mono VO (no IMU, no backend) ---
add_executable(euroc_mono_vo euroc_mono_vo.cpp)
target_link_libraries(euroc_mono_vo PRIVATE ${EUROC_COMMON_LIBS})
