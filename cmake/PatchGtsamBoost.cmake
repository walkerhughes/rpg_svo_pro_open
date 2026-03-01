# Helper script: Patch GTSAM 4.2's HandleBoost.cmake for Boost >= 1.69
# where boost_system became header-only (no compiled library).
#
# Usage (run from GTSAM source root):
#   cmake -P /path/to/PatchGtsamBoost.cmake
file(READ cmake/HandleBoost.cmake _content)

# Remove 'system' from the find_package components list
string(REPLACE
  "serialization system filesystem thread"
  "serialization filesystem thread"
  _content "${_content}")

# Remove Boost_SYSTEM_LIBRARY from the required-components check
string(REPLACE
  "NOT Boost_SYSTEM_LIBRARY OR NOT Boost_FILESYSTEM_LIBRARY"
  "NOT Boost_FILESYSTEM_LIBRARY"
  _content "${_content}")

# Remove Boost::system from the link libraries list
string(REPLACE
  "  Boost::system\n"
  ""
  _content "${_content}")

file(WRITE cmake/HandleBoost.cmake "${_content}")
message(STATUS "Patched GTSAM HandleBoost.cmake: removed boost_system requirement")
