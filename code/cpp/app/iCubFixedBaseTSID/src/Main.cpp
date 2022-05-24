/**
 * @file Main.cpp
 * @authors Ines Sorrentino <ines.sorrentino@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <iCub-torque-control/iCubFixedBaseTSID/Module.h>

int main(int argc, char* argv[])
{
    // initialize yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "[main] Unable to find YARP network";
        return EXIT_FAILURE;
    }

    BipedalLocomotion::System::ClockBuilder::setFactory(std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

    // prepare and configure the resource finder
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    rf.setDefaultConfigFile("iCub-fixedbaseTSID-options.ini");

    rf.configure(argc, argv);

    // create the module
    iCubTorqueControl::iCubFixedBaseTSID::Module module;

    return module.runModule(rf);
}
