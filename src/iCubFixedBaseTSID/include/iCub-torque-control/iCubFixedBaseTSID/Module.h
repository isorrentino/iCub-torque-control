/**
 * @file Module.h
 * @authors Ines Sorrentino
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_UTILITIES_JOINT_TRAJECTORY_PLAYER_MODULE_H
#define BIPEDAL_LOCOMOTION_UTILITIES_JOINT_TRAJECTORY_PLAYER_MODULE_H

// std
#include <deque>
#include <memory>
#include <string>
#include <vector>

// YARP
#include <yarp/os/RFModule.h>

#include <matioCpp/matioCpp.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/TSID/QPFixedBaseTSID.h>
#include <BipedalLocomotion/TSID/JointTrackingTask.h>
#include <BipedalLocomotion/TSID/SE3Task.h>
#include <BipedalLocomotion/Planners/SwingFootPlanner.h>
#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>

// iDynTree
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>


using namespace BipedalLocomotion;
using namespace BipedalLocomotion::TSID;
using namespace BipedalLocomotion::ContinuousDynamicalSystem;

namespace iCubTorqueControl
{
namespace iCubFixedBaseTSID
{

class Module : public yarp::os::RFModule
{
    double m_dT; /**< RFModule period. */

    std::string m_robot; /**< Robot name. */

    int m_numOfJoints; /**< Number of joints to control. */

    std::vector<std::string> m_jointNamesList; /**< Joint names list. */

    BipedalLocomotion::RobotInterface::PolyDriverDescriptor m_controlBoard; /**< Control board remapper. */

    BipedalLocomotion::RobotInterface::YarpRobotControl m_robotControl; /**< Robot control object. */

    BipedalLocomotion::RobotInterface::YarpSensorBridge m_sensorBridge; /**< Sensor bridge object. */

    iDynTree::ModelLoader m_loader; /**< Model loader object. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< kinDynComputation object. */

    BipedalLocomotion::Planners::SwingFootPlanner m_planner;

    Eigen::Vector3d m_gravity;

    struct TSIDAndTasks
    {
        std::shared_ptr<QPFixedBaseTSID> tsid;
        std::shared_ptr<SE3Task> se3Task;
        std::shared_ptr<JointTrackingTask> regularizationTask;
    };

    TSIDAndTasks m_tsidAndTasks;

    Eigen::VectorXd m_currentJointPos; /**< Current joint positions. */
    Eigen::VectorXd m_currentJointVel; /**< Current joint velocities. */
    Eigen::VectorXd m_desJointTorque; /**< Desired joint torques. */
    Eigen::VectorXd m_desJointPos; /**< Desired joint positions. */
    Eigen::VectorXd m_desJointVel; /**< Desired joint velocities. */
    Eigen::VectorXd m_desJointAcc; /**< Desired joint accelerations. */
    manif::SE3d m_currentEEPos; /**< Current end-effector position */
    Eigen::VectorXd m_currentJointTrq; /**< Current joint torques. */

    BipedalLocomotion::Contacts::ContactList m_contactList;

    std::string m_controlledFrame;

    std::unordered_map<std::string, std::vector<double>> m_log; /**< Measured joint and motor quantities. */

    struct AcccelerationIntegrator
    {
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<
            BipedalLocomotion::ContinuousDynamicalSystem::LinearTimeInvariantSystem>>
            integrator;
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::LinearTimeInvariantSystem>
            dynamics;
    };
    AcccelerationIntegrator m_accSystem;


    // Private methods
    bool createPolydriver(std::shared_ptr<ParametersHandler::IParametersHandler> handler);

    bool initializeRobotControl(std::shared_ptr<ParametersHandler::IParametersHandler> handler);

    bool instantiateSensorBridge(std::shared_ptr<ParametersHandler::IParametersHandler> handler);

    bool setRobotModel(const yarp::os::Searchable& rf);

    bool createFixedBaseTSID(std::shared_ptr<ParametersHandler::IParametersHandler> handler);

    void logData();


public:
    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() override;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() override;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;
};
} // namespace iCubFixedBaseTSID
} // namespace iCubTorqueControl

#endif // BIPEDAL_LOCOMOTION_UTILITIES_JOINT_TRAJECTORY_PLAYER_MODULE_H

