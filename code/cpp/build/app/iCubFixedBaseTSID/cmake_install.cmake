# Install script for directory: /home/isorrentino/dev/iCub-torque-control/code/cpp/app/iCubFixedBaseTSID

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/isorrentino/dev/iCub-torque-control/code/cpp/build/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xbinx" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/iCubFixedBaseTSID-0.0.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/iCubFixedBaseTSID"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "$ORIGIN/:$ORIGIN/../lib:/home/isorrentino/dev/robotology-superbuild/build/install/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES
    "/home/isorrentino/dev/iCub-torque-control/code/cpp/build/bin/iCubFixedBaseTSID-0.0.1"
    "/home/isorrentino/dev/iCub-torque-control/code/cpp/build/bin/iCubFixedBaseTSID"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/iCubFixedBaseTSID-0.0.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/iCubFixedBaseTSID"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/isorrentino/dev/robotology-superbuild/build/install/lib::::::::::::::::::::::::"
           NEW_RPATH "$ORIGIN/:$ORIGIN/../lib:/home/isorrentino/dev/robotology-superbuild/build/install/lib")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/BipedalLocomotionFramework/robots/iCubGazeboV3" TYPE FILE FILES
    "/home/isorrentino/dev/iCub-torque-control/code/cpp/app/iCubFixedBaseTSID/config/robots/iCubGazeboV3/iCub-fixedbaseTSID-options.ini"
    "/home/isorrentino/dev/iCub-torque-control/code/cpp/app/iCubFixedBaseTSID/config/robots/iCubGazeboV3/planner.ini"
    "/home/isorrentino/dev/iCub-torque-control/code/cpp/app/iCubFixedBaseTSID/config/robots/iCubGazeboV3/tsid.ini"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/BipedalLocomotionFramework/robots/iCubGenova04" TYPE FILE FILES
    "/home/isorrentino/dev/iCub-torque-control/code/cpp/app/iCubFixedBaseTSID/config/robots/iCubGenova04/iCub-fixedbaseTSID-options.ini"
    "/home/isorrentino/dev/iCub-torque-control/code/cpp/app/iCubFixedBaseTSID/config/robots/iCubGenova04/planner.ini"
    "/home/isorrentino/dev/iCub-torque-control/code/cpp/app/iCubFixedBaseTSID/config/robots/iCubGenova04/tsid.ini"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/BipedalLocomotionFramework/robots/iCubGenova09" TYPE FILE FILES
    "/home/isorrentino/dev/iCub-torque-control/code/cpp/app/iCubFixedBaseTSID/config/robots/iCubGenova09/iCub-fixedbaseTSID-options.ini"
    "/home/isorrentino/dev/iCub-torque-control/code/cpp/app/iCubFixedBaseTSID/config/robots/iCubGenova09/planner.ini"
    "/home/isorrentino/dev/iCub-torque-control/code/cpp/app/iCubFixedBaseTSID/config/robots/iCubGenova09/tsid.ini"
    )
endif()

