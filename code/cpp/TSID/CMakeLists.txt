# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Authors: Ines Sorrentino <ines.sorrentino@iit.it>

cmake_minimum_required(VERSION 3.14)

## MAIN project
project(iCubTorqueControl
  VERSION 0.0.1)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

# Check iCubTorqueControl dependencies, find necessary libraries.
include(iCubTorqueControlDependencies)

#set default build type to "Release" in single-config generators
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, recommanded options are: Debug or Release" FORCE)
    endif()
    set(ROBOTOLOGY_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${ROBOTOLOGY_BUILD_TYPES})
endif()

# Defines the CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR and many other useful macros.
# See https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
# With the following settings executables are placed in <the top level of the
# build tree>/bin and libraries/archives in <top level of the build tree>/lib.
# This is particularly useful to run ctests on libraries built on Windows
# machines: tests, which are executables, are placed in the same folders of
# dlls, which are treated as executables as well, so that they can properly
# find the libraries to run. This is a because of missing RPATH on Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE.
# See https://cmake.org/cmake/help/v3.4/variable/CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS.html
# See https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Under MSVC, we set CMAKE_DEBUG_POSTFIX to "d" to add a trailing "d" to library
# built in debug mode. In this Windows user can compile, build and install the
# library in both Release and Debug configuration avoiding naming clashes in the
# installation directories.
if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

#Function to automatize the process of creating a new application
include(AddiCubTorqueControlApplication)

#Utility to install ini files
include(InstallIniFiles)

add_subdirectory(src)

#Creates the interface target BipedalLocomotion::Framework that depends on all the targets.
#By including the file "BipedalLocomotion/Framework.h" it is possible to include all the headers automatically.
include(AddiCubTorqueControlTarget)

include(InstallBasicPackageFiles)
get_property(PRIVATE_DEPENDENCIES GLOBAL PROPERTY iCubTorqueControl_PrivateDependencies)
get_property(PUBLIC_DEPENDENCIES GLOBAL PROPERTY iCubTorqueControl_PublicDependencies)
install_basic_package_files(${PROJECT_NAME}
                            NAMESPACE iCubTorqueControl::
                            VERSION ${iCubTorqueControl_VERSION}
                            COMPATIBILITY AnyNewerVersion
                            TARGETS_PROPERTY iCubTorqueControl_TARGETS
                            VARS_PREFIX ${PROJECT_NAME}
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
                            DEPENDENCIES ${PUBLIC_DEPENDENCIES}
                            PRIVATE_DEPENDENCIES ${PRIVATE_DEPENDENCIES}
                            OVERRIDE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake)


include(AddUninstallTarget)
