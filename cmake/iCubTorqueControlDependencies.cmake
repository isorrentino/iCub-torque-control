# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include(iCubTorqueControlFindDependencies)
include(iCubTorqueControlDependencyClassifier)

################################################################################
########################## Mandatory dependencies ##############################

find_package(BipedalLocomotionFramework 0.1.0 REQUIRED)
dependency_classifier(BipedalLocomotionFramework MINIMUM_VERSION 0.1.0 IS_USED TRUE PUBLIC)

find_package(iDynTree 3.0.0 REQUIRED)
dependency_classifier(iDynTree MINIMUM_VERSION 3.0.0 IS_USED TRUE PUBLIC)

find_package(Eigen3 3.2.92 REQUIRED)
dependency_classifier(Eigen3 MINIMUM_VERSION 3.2.92 IS_USED TRUE PUBLIC)

########################## Optional dependencies ##############################

find_package(YARP QUIET)
checkandset_dependency(YARP)
dependency_classifier(YARP IS_USED ${FRAMEWORK_USE_YARP} PUBLIC)

find_package(OsqpEigen 0.6.3 QUIET)
checkandset_dependency(OsqpEigen)
dependency_classifier(OsqpEigen MINIMUM_VERSION 0.6.3 IS_USED ${FRAMEWORK_USE_OsqpEigen})

find_package(matioCpp QUIET)
checkandset_dependency(matioCpp)
dependency_classifier(matioCpp IS_USED ${FRAMEWORK_USE_matioCpp} PUBLIC)

