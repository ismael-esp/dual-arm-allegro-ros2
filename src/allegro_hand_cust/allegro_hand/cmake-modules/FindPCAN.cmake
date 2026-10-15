# - Try to find peak-linux driver
# Once done, this will define
#
#  PCAN_FOUND - system has pcan
#  PCAN_INCLUDE_DIR - the libpcan include directories
#  PCAN_LIBRARY - the libpcan library to link
#  PCANFD_LIBRARY - the libpcanfd library to link

IF(UNIX)
# Include dir
    find_path(PCAN_INCLUDE_DIR libpcan.h
        HINTS /usr/include /usr/include/pcan)

#Library Path
    find_library(PCAN_LIBRARY
	    NAMES libpcan.so
	    PATHS /usr/lib /usr/local/lib
	    )
ENDIF(UNIX)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBPCAN_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(PCAN DEFAULT_MSG
                                  PCAN_LIBRARY PCAN_INCLUDE_DIR)

mark_as_advanced(PCAN_INCLUDE_DIR PCAN_LIBRARY)

set(PCAN_INCLUDE_DIRS ${PCAN_INCLUDE_DIR})
set(PCAN_LIBRARIES ${PCAN_LIBRARY})
