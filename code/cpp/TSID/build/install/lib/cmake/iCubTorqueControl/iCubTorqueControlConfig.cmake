set(iCubTorqueControl_VERSION 0.0.1)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was iCubTorqueControlConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

####################################################################################

#### Expanded from @PACKAGE_DEPENDENCIES@ by install_basic_package_files() ####

include(CMakeFindDependencyMacro)
set(CMAKE_MODULE_PATH_BK ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH /home/isorrentino/dev/iCub-torque-control/cmake)
find_dependency(BipedalLocomotionFramework 0.1.0)
find_dependency(iDynTree 3.0.0)
find_dependency(Eigen3 3.2.92)
find_dependency(YARP)
find_dependency(matioCpp)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH_BK})

###############################################################################


if(NOT TARGET iCubTorqueControl::TorqueControlApplications)
  include("${CMAKE_CURRENT_LIST_DIR}/iCubTorqueControlTargets.cmake")
endif()

# Compatibility
get_property(iCubTorqueControl_TorqueControlApplications_INCLUDE_DIR TARGET iCubTorqueControl::TorqueControlApplications PROPERTY INTERFACE_INCLUDE_DIRECTORIES)

set(iCubTorqueControl_LIBRARIES iCubTorqueControl::TorqueControlApplications)
set(iCubTorqueControl_INCLUDE_DIRS "${iCubTorqueControl_TorqueControlApplications_INCLUDE_DIR}")
list(REMOVE_DUPLICATES iCubTorqueControl_INCLUDE_DIRS)


