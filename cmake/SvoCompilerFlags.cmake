add_library(svo_compiler_flags INTERFACE)
add_library(svo::compiler_flags ALIAS svo_compiler_flags)

# Platform-aware SIMD flags
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mfpu=neon" HAS_NEON)
if(HAS_NEON)
  target_compile_options(svo_compiler_flags INTERFACE -mfpu=neon)
else()
  target_compile_options(svo_compiler_flags INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-msse2>
  )
endif()

target_compile_options(svo_compiler_flags INTERFACE
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wno-unused-parameter>
)

# Compile definitions
target_compile_definitions(svo_compiler_flags INTERFACE
  SVO_USE_OPENGV
  SVO_DEPTHFILTER_IN_REPROJECTOR
  GLOG_USE_GLOG_EXPORT
  $<$<BOOL:${SVO_BUILD_LOOP_CLOSING}>:SVO_LOOP_CLOSING>
  $<$<BOOL:${SVO_BUILD_GLOBAL_MAP}>:SVO_GLOBAL_MAP>
)
