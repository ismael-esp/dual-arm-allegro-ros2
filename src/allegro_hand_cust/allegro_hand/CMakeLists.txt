#     Copyright 2024 Institute of Industrial and Control Engineering (IOC)
#                  Universitat Politecnica de Catalunya
#                  BarcelonaTech
#     All Rights Reserved.
#
#     This program is free software; you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation; either version 2 of the License, or
#     (at your option) any later version.
#
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#     along with this program; if not, write to the
#     Free Software Foundation, Inc.,
#     59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
#
#    Author:   Leopold Palomo-Avellaneda
#
##############################################################################
# Project info
##############################################################################

cmake_minimum_required(VERSION 3.3.1)
cmake_policy(SET CMP0057 NEW)
project(allegro_hand VERSION 0.0.1 LANGUAGES CXX)

set(PROJECT_NAME allegro_hand)
set(PROJECT_BUGREPORT leopold.palomo@upc.edu)
set(
  PROJECT_STRING
  "${PROJECT_NAME} ${PROJECT_VERSION}"
  "Library to read data from and command configurations to the Allegro Hand"
)
set(PROJECT_TARNAME ${PROJECT_NAME})
set(PROJECT_NAME_VERSION
  ${PROJECT_NAME_MAJOR_VERSION}.${PROJECT_NAME_MINOR_VERSION}.${PROJECT_NAME_PATCH_VERSION})

# some needed stuff
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)
message(STATUS "Using CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")

# some definitions
add_definitions(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Required packages
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
if(EIGEN3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
    set(ALLEGROHAND_LIBS ${ALLEGROHAND_LIBS} Eigen3::Eigen)
    message(STATUS "Eigen found. Added ${EIGEN3_INCLUDE_DIR} as includes")
else(EIGEN3_FOUND)
    message (CRITICAL "AllegroHandDriver lib needs Eigen3 to be built")
endif(EIGEN3_FOUND)

find_package(yaml-cpp REQUIRED)
if(yaml-cpp_FOUND)
    include_directories(${YAML_INCLUDE_DIR})
    set(ALLEGROHAND_LIBS ${ALLEGROHAND_LIBS} ${YAML_CPP_LIBRARIES})
else(yaml-cpp_FOUND)
    message (CRITICAL "AllegroHandDriver lib needs yaml-cpp to be built")
endif(yaml-cpp_FOUND)

find_package(PCANBasic REQUIRED MODULE)

if(PCANBasic_FOUND)
    include_directories(${PCANBasic_INCLUDE_DIR})
    set(ALLEGROHAND_LIBS ${ALLEGROHAND_LIBS} ${PCANBasic_LIBRARY})
    message (STATUS "Peak system libpcan ${PCANBasic_INCLUDE_DIR} and ${PCANBasic_LIBRARY}")
else(PCANBasic_FOUND)
    message (CRITICAL "AllegroHandDriver lib needs Peak system libpcan to be built")
endif(PCANBasic_FOUND)


find_package(orocos_kdl REQUIRED)
if(orocos_kdl_FOUND)
        message(STATUS "orocos_kdl found. Version: ${orocos_kdl_VERSION}")
        include_directories(${orocos_kdl_INCLUDE_DIRS})
        add_definitions(${orocos_kdl_CFLAGS})
        set(ALLEGROHAND_LIBS ${ALLEGROHAND_LIBS} ${orocos_kdl_LIBRARIES})
        link_directories(${orocos_kdl_LIBRARY_DIR})
endif(orocos_kdl_FOUND)

find_package(Boost REQUIRED COMPONENTS system filesystem)
if(Boost_FOUND)
        message(STATUS "Boost found. Version: ${Boost_VERSION}")
        include_directories(${Boost_INCLUDE_DIRS})
        add_definitions(${Boost_CFLAGS})
        set(ALLEGROHAND_LIBS ${ALLEGROHAND_LIBS} ${Boost_LIBRARIES})
        link_directories(${Boost_LIBRARY_DIR})
endif(Boost_FOUND)

#if(NOT DEFINED BUILD_SHARED_LIBS)
#  option(BUILD_SHARED_LIBS "Build dynamically-linked binaries" ON)
#endif()

find_package(urdf REQUIRED)
if(urdf_FOUND)
        message(STATUS "urdf found. Version: ${urdf_VERSION}")
        include_directories(${urdf_INCLUDE_DIRS})
        add_definitions(${urdf_CFLAGS})
        set(ALLEGROHAND_LIBS ${ALLEGROHAND_LIBS}  ${urdf_LIBRARIES})
        link_directories(${urdf_LIBRARY_DIR})
endif(urdf_FOUND)

find_library(TinyXML NAMES tinyxml)

find_package(Filesystem REQUIRED)

# thread lib
find_package( Threads REQUIRED)
set( THREADS_PREFER_PTHREAD_FLAG TRUE)

set(ALLEGROHAND_LIBS ${ALLEGROHAND_LIBS} ${TINYXML_LIBRARIES} ${TinyXML} ${CMAKE_THREAD_LIBS_INIT} std::filesystem )


#Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
message(STATUS "Project source dir: ${PROJECT_SOURCE_DIR}")


##############################################################################
# Subdirectories
##############################################################################

#Library Allegro
add_subdirectory (src)

# Build basic example, test program
add_subdirectory (examples)

##############################################################################
# File describing how to use library
##############################################################################

#############
## Install ##
#############
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)


##############################################################################
# File describing how to use library
##############################################################################
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)

# Create the ${PROJECT_NAME}Config.cmake.
configure_file(
  ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}Config.cmake.in
  "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY
)

# Create the ${PROJECT_NAME}ConfigVersion.cmake.
write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY AnyNewerVersion
)

# cmake stuff files
install(
  FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

# Export targets.
set(export_targets ${export_targets};${PROJECT_NAME})
export(
  EXPORT export_${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
  NAMESPACE ${PROJECT_NAME}::
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
)

# Install conf files
install(DIRECTORY conf DESTINATION share/allegro_hand)

# package xml
install(
  FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/package.xml"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/
)
