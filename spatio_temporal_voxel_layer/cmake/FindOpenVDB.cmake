# FindOpenVDB.cmake - Find OpenVDB library
#
# Sets:
#   OpenVDB_FOUND
#   OpenVDB_INCLUDE_DIRS
#   OpenVDB_LIBRARIES

find_path(OpenVDB_INCLUDE_DIR
  NAMES openvdb/openvdb.h
  PATHS /usr/include /usr/local/include
)

find_library(OpenVDB_LIBRARY
  NAMES openvdb
  PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenVDB
  REQUIRED_VARS OpenVDB_LIBRARY OpenVDB_INCLUDE_DIR
)

if(OpenVDB_FOUND)
  set(OpenVDB_INCLUDE_DIRS ${OpenVDB_INCLUDE_DIR})
  set(OpenVDB_LIBRARIES ${OpenVDB_LIBRARY})
endif()
