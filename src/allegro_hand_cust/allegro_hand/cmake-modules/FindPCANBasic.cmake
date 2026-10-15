# - Try to find peak-linux driver
# Once done, this will define
#
#  PCANBasic_FOUND - system has pcan
#  PCANBasic_INCLUDE_DIR - the libpcan include directories
#  PCANBasic_LIBRARY - the libpcan library to link
#  PCANFD_LIBRARY - the libpcanfd library to link

IF(UNIX)
# Include dir
    find_path(PCANBasic_INCLUDE_DIR PCANBasic.h
        HINTS /usr/include /usr/include/pcan)

#Library Path
    find_library(PCANBasic_LIBRARY
	    NAMES libpcanbasic.so
	    PATHS /usr/lib /usr/local/lib
	    )
ENDIF(UNIX)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBPCAN_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(PCANBasic DEFAULT_MSG
                                  PCANBasic_LIBRARY PCANBasic_INCLUDE_DIR)

mark_as_advanced(PCANBasic_INCLUDE_DIR PCANBasic_LIBRARY)

set(PCANBasic_INCLUDE_DIRS ${PCANBasic_INCLUDE_DIR})
set(PCANBasic_LIBRARIES ${PCANBasic_LIBRARY})
