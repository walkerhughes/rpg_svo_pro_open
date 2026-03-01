add_library(fast
  src/fast_10.cpp
  src/fast_10_score.cpp
  src/fast_9.cpp
  src/nonmax_3x3.cpp
  src/nonmax_5x5.cpp
)

# Only include SSE on x86, NEON on ARM
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|i[3-6]86")
  target_sources(fast PRIVATE src/faster_corner_10_sse.cpp)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|arm")
  target_sources(fast PRIVATE src/faster_corner_9_neon.cpp)
  # Suppress C++17 'register' keyword error in legacy NEON code
  set_source_files_properties(src/faster_corner_9_neon.cpp PROPERTIES
    COMPILE_FLAGS "-Wno-register")
endif()

add_library(svo::fast ALIAS fast)

target_include_directories(fast PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
