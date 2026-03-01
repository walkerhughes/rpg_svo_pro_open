add_library(svo_compiler_flags INTERFACE)
add_library(svo::compiler_flags ALIAS svo_compiler_flags)

# Platform-aware SIMD flags
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|arm")
  # ARM: NEON is available natively on AArch64 (no flag needed).
  # 32-bit ARM may need -mfpu=neon.
  if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-mfpu=neon" HAS_NEON)
    if(HAS_NEON)
      target_compile_options(svo_compiler_flags INTERFACE -mfpu=neon)
    endif()
  endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|i[3-6]86")
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
