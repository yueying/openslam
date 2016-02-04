# ----------------------------------------------------------------------------
#  Update the library version header file
#    FILE_TO_PARSE="SRC/include/openslam/OPENSLAM_version.h.in"
#    TARGET_FILE  ="OPENSLAM_version.h"
# ----------------------------------------------------------------------------
SET(CMAKE_OPENSLAM_COMPLETE_NAME "OPENSLAM ${CMAKE_OPENSLAM_VERSION_NUMBER_MAJOR}.${CMAKE_OPENSLAM_VERSION_NUMBER_MINOR}.${CMAKE_OPENSLAM_VERSION_NUMBER_PATCH}")
# Build a three digits version code, eg. 0.5.1 -> 051,  1.2.0 -> 120
SET(CMAKE_OPENSLAM_VERSION_CODE "0x${CMAKE_OPENSLAM_VERSION_NUMBER_MAJOR}${CMAKE_OPENSLAM_VERSION_NUMBER_MINOR}${CMAKE_OPENSLAM_VERSION_NUMBER_PATCH}")

CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/version.h.in" "${OPENSLAM_CONFIG_FILE_INCLUDE_DIR}/openslam/version.h")

