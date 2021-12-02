# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Authors: Ines Sorrentino


add_iCub_torque_control_application(
    NAME iCubFixedBaseTSID
    SOURCES src/Main.cpp src/Module.cpp
    HEADERS include/iCub-torque-control/iCubFixedBaseTSID/Module.h
    LINK_LIBRARIES  YARP::YARP_dev
    BipedalLocomotion::ParametersHandlerYarpImplementation
    BipedalLocomotion::RobotInterfaceYarpImplementation
    BipedalLocomotion::Planners
    matioCpp::matioCpp
    BipedalLocomotion::matioCppConversions
    BipedalLocomotion::TSID
    BipedalLocomotion::SystemYarpImplementation
    iDynTree::idyntree-high-level
    iDynTree::idyntree-model
    )

install_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/config)

