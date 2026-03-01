cmake_minimum_required(VERSION 3.16...4.1)
project(svo_pro VERSION 1.0.0 LANGUAGES CXX)

# Allow fetched dependencies with old cmake_minimum_required (opengv 3.1, DBoW2 3.0).
# Without this, CMake >= 4.0 rejects cmake_minimum_required(VERSION < 3.5).
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING
  "Minimum policy version for compatibility with old third-party CMakeLists")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(SVO_BUILD_TESTS "Build unit tests" OFF)
option(SVO_BUILD_LOOP_CLOSING "Build loop closing modules" ON)
option(SVO_BUILD_GLOBAL_MAP "Build global map modules (requires GTSAM)" OFF)

# Compiler flags
include(cmake/SvoCompilerFlags.cmake)

# Required dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres 2.2 REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP QUIET)

if(SVO_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
endif()

# Optional dependencies via FetchContent
include(FetchContent)

# opengv
find_package(opengv QUIET)
if(NOT opengv_FOUND)
  # Help opengv's old FindEigen.cmake find Eigen
  get_target_property(_eigen_inc Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  set(EIGEN_INCLUDE_DIR "${_eigen_inc}" CACHE PATH "Eigen include dir for opengv")
  set(EIGEN_INCLUDE_DIRS "${_eigen_inc}" CACHE PATH "Eigen include dirs for opengv")
  set(EIGEN_VERSION "5.0.1" CACHE STRING "Eigen version")
  set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_PYTHON OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(opengv
    GIT_REPOSITORY https://github.com/laurentkneip/opengv.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
  )
  FetchContent_MakeAvailable(opengv)
  # opengv defaults to C++11 but Eigen 5 requires C++17
  if(TARGET opengv)
    set_target_properties(opengv PROPERTIES
      CXX_STANDARD 17
      CXX_STANDARD_REQUIRED ON)
    # opengv code uses assert() without including <cassert>
    target_compile_options(opengv PRIVATE -include cassert)
  endif()
  # random_generators is unconditionally built by opengv
  if(TARGET random_generators)
    set_target_properties(random_generators PROPERTIES
      CXX_STANDARD 17
      CXX_STANDARD_REQUIRED ON)
    target_compile_options(random_generators PRIVATE -include cassert)
  endif()
endif()

# DBoW2
find_package(DBoW2 QUIET)
if(NOT DBoW2_FOUND)
  FetchContent_Declare(DBoW2
    GIT_REPOSITORY https://github.com/dorian3d/DBoW2.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
  )
  FetchContent_MakeAvailable(DBoW2)
endif()

# Third-party vendored libraries
add_subdirectory(third_party/minkindr)
add_subdirectory(third_party/eigen_checks)
add_subdirectory(third_party/fast_neon)

# Libraries in dependency order
add_subdirectory(rpg_common)
add_subdirectory(vikit/vikit_solver)
add_subdirectory(vikit/vikit_common)
add_subdirectory(vikit/vikit_cameras)
add_subdirectory(svo_vio_common)
add_subdirectory(svo_common)
add_subdirectory(svo_direct)
add_subdirectory(svo_img_align)
add_subdirectory(svo_tracker)
add_subdirectory(svo_test_utils)
add_subdirectory(svo_ceres_backend)

if(SVO_BUILD_LOOP_CLOSING)
  add_subdirectory(svo_pgo)
  add_subdirectory(svo_online_loopclosing)
endif()

add_subdirectory(svo)

# Collect all library targets for install
set(SVO_INSTALL_TARGETS
  svo_compiler_flags
  minkindr
  eigen_checks
  fast
  rpg_common
  vikit_solver
  vikit_common
  vikit_cameras
  svo_vio_common
  svo_common
  svo_direct
  svo_img_align
  svo_tracker
  svo_test_utils
  svo_ceres_backend
  svo
)
if(SVO_BUILD_LOOP_CLOSING)
  list(APPEND SVO_INSTALL_TARGETS svo_online_loopclosing svo_pgo)
endif()

install(TARGETS ${SVO_INSTALL_TARGETS}
  EXPORT svo-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install headers for each package
install(DIRECTORY rpg_common/include/ DESTINATION include)
install(DIRECTORY vikit/vikit_solver/include/ DESTINATION include)
install(DIRECTORY vikit/vikit_common/include/ DESTINATION include)
install(DIRECTORY vikit/vikit_cameras/include/ DESTINATION include)
install(DIRECTORY svo_vio_common/include/ DESTINATION include)
install(DIRECTORY svo_common/include/ DESTINATION include)
install(DIRECTORY svo_direct/include/ DESTINATION include)
install(DIRECTORY svo_img_align/include/ DESTINATION include)
install(DIRECTORY svo_tracker/include/ DESTINATION include)
install(DIRECTORY svo_test_utils/include/ DESTINATION include)
install(DIRECTORY svo_ceres_backend/include/ DESTINATION include)
install(DIRECTORY svo/include/ DESTINATION include)
install(DIRECTORY third_party/minkindr/include/ DESTINATION include)
install(DIRECTORY third_party/eigen_checks/include/ DESTINATION include)
if(SVO_BUILD_LOOP_CLOSING)
  install(DIRECTORY svo_online_loopclosing/include/ DESTINATION include)
  install(DIRECTORY svo_pgo/include/ DESTINATION include)
endif()

# Install CMake config files
include(CMakePackageConfigHelpers)
install(EXPORT svo-targets
  FILE svo-targets.cmake
  NAMESPACE svo::
  DESTINATION lib/cmake/svo
)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/svo-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/svo-config.cmake
  INSTALL_DESTINATION lib/cmake/svo
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/svo-config-version.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/svo-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/svo-config-version.cmake
  DESTINATION lib/cmake/svo
)
