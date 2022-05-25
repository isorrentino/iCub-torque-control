/**
 * @file Module.cpp
 * @authors Ines Sorrentino <ines.sorrentino@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iomanip>
#include <iCubFixedBaseTSID/Module.h>

// BipedalLocomotionFramework
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/Conversions/matioCppConversions.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::TSID;
using namespace iCubTorqueControl::iCubFixedBaseTSID;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Contacts;
using namespace BipedalLocomotion::Planners;
using namespace BipedalLocomotion::ContinuousDynamicalSystem;

double m_time = 0;

double Module::getPeriod()
{
    return m_dT;
}

bool Module::createFixedBaseTSID(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    std::string robotAccelerationName;
    if (!handler->getParameter("robot_acceleration_variable_name",robotAccelerationName))
    {
        std::cerr << "[Module::createFixedBaseTSID] robot_acceleration_variable_name parameter not found.";
        return false;
    }

    std::string jointTorqueName;
    if (!handler->getParameter("joint_torques_variable_name",jointTorqueName))
    {
        std::cerr << "[Module::createFixedBaseTSID] joint_torques_variable_name parameter not found.";
        return false;
    }

    // Instantiate the handler
    System::VariablesHandler variablesHandler;
    if (!variablesHandler.addVariable(robotAccelerationName, m_numOfJoints + 6))
    {
        std::cerr << "[Module::createFixedBaseTSID] Impossible to add variable robotAccelerationName to the variable handler.";
        return false;
    }
    if (!variablesHandler.addVariable(jointTorqueName, m_numOfJoints))
    {
        std::cerr << "[Module::createFixedBaseTSID] Impossible to add variable jointTorqueName to the variable handler.";
        return false;
    }

    //// Create TSID object
    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;
    m_tsidAndTasks.tsid = std::make_shared<QPFixedBaseTSID>();
    if (!m_tsidAndTasks.tsid->setKinDyn(m_kinDyn))
    {
        std::cerr << "[Module::createFixedBaseTSID] Impossible to set kinDyn for the tsid object.";
        return false;
    }
    if (!m_tsidAndTasks.tsid->initialize(handler))
    {
        std::cerr << "[Module::createFixedBaseTSID] Impossible to initialize the tsid object.";
        return false;
    }

    //// Initialize SE3 Task
    m_tsidAndTasks.se3Task = std::make_shared<SE3Task>();
    if (!m_tsidAndTasks.se3Task->setKinDyn(m_kinDyn))
    {
        std::cerr << "[Module::createFixedBaseTSID] Impossible to set kinDyn for the se3task.";
        return false;
    }
    if (!m_tsidAndTasks.se3Task->initialize(handler->getGroup("EE_SE3_TASK")))
    {
        std::cerr << "[Module::createFixedBaseTSID] Impossible to initialize the se3task.";
        return false;
    }
    Eigen::VectorXd weight_se3;
    handler->getGroup("EE_SE3_TASK").lock()->getParameter("weight",weight_se3);
    if (!m_tsidAndTasks.tsid->addTask(m_tsidAndTasks.se3Task, "se3_task", lowPriority, weight_se3))
    //if (!m_tsidAndTasks.tsid->addTask(m_tsidAndTasks.se3Task, "se3_task", highPriority))
    {
        std::cerr << "[Module::createFixedBaseTSID] Impossible to add the se3task to the tsid.";
        return false;
    }

    //// Initialize regularization task
    m_tsidAndTasks.regularizationTask = std::make_shared<JointTrackingTask>();
    if (!m_tsidAndTasks.regularizationTask->setKinDyn(m_kinDyn))
    {
        std::cerr << "[Module::createFixedBaseTSID] Impossible to set kinDyn for the regularization task.";
        return false;
    }
    if (!m_tsidAndTasks.regularizationTask->initialize(handler->getGroup("JOINT_REGULARIZATION_TASK")))
    {
        std::cerr << "[Module::createFixedBaseTSID] Impossible to initialize the regularization task.";
        return false;
    }
    Eigen::VectorXd weight_regulatization;
    handler->getGroup("JOINT_REGULARIZATION_TASK").lock()->getParameter("weight",weight_regulatization);
    if (!m_tsidAndTasks.tsid->addTask(m_tsidAndTasks.regularizationTask,
                            "regularization_task",
                            lowPriority,
                            weight_regulatization))
    {
        std::cerr << "[Module::createFixedBaseTSID] Impossible to add the regularization task to the tsid.";
        return false;
    }

    if (!m_tsidAndTasks.tsid->finalize(variablesHandler))
    {
        std::cout << variablesHandler.toString() << std::endl;
        std::cerr << "Unable to finalize the TSID." << std::endl;
        return false;
    }

    return true;
}

bool Module::createPolydriver(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    auto ptr = handler->getGroup("ROBOT_INTERFACE").lock();
    if (ptr == nullptr)
    {
        std::cerr << "[Module::createPolydriver] Robot interface options is empty." << std::endl;
        return false;
    }
    ptr->setParameter("local_prefix", this->getName());
    m_controlBoard = RobotInterface::constructRemoteControlBoardRemapper(ptr);
    if (!m_controlBoard.isValid())
    {
        std::cerr << "[Module::createPolydriver] the robot polydriver has not been constructed."
                  << std::endl;
        return false;
    }

    return true;
}

bool Module::initializeRobotControl(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    if (!m_robotControl.initialize(handler->getGroup("ROBOT_CONTROL")))
    {
        std::cerr << "[Module::initializeRobotControl] Unable to initialize the "
                     "control board"
                  << std::endl;
        return false;
    }
    if (!m_robotControl.setDriver(m_controlBoard.poly))
    {
        std::cerr << "[Module::initializeRobotControl] Unable to initialize the "
                     "control board"
                  << std::endl;
        return false;
    }

    return true;
}

bool Module::instantiateSensorBridge(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    if (!m_sensorBridge.initialize(handler->getGroup("SENSOR_BRIDGE")))
    {
        std::cerr << "[Module::initializeSensorBridge] Unable to initialize the sensor bridge"
                  << std::endl;
        return false;
    }

    yarp::dev::PolyDriverList list;
    list.push(m_controlBoard.poly.get(), m_controlBoard.key.c_str());
    if (!m_sensorBridge.setDriversList(list))
    {
        std::cerr << "[Module::initializeSensorBridge] Unable to set the driver list" << std::endl;
        return false;
    }

    return true;
}

bool Module::setRobotModel(const yarp::os::Searchable& rf)
{
    if(m_jointNamesList.empty())
    {
        std::cerr << "[Module::setRobotModel] The list containing the controlled joints is empty. "
                 <<  "Please call setControlledJoints()";
        return false;
    }

    // load the model in iDynTree::KinDynComputations
    std::string model = rf.check("model",yarp::os::Value("model.urdf")).asString();
    std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(model);

    std::cout << "The model is found in: " << pathToModel;

    // only the controlled joints are extracted from the URDF file
    if(!m_loader.loadReducedModelFromFile(pathToModel, m_jointNamesList))
    {
        std::cerr << "[Module::setRobotModel] Error while loading the model from " << pathToModel;
        return false;
    }
    return true;
}

bool Module::configure(yarp::os::ResourceFinder& rf)
{
    auto parametersHandler = std::make_shared<ParametersHandler::YarpImplementation>(rf);

    std::string name;
    if (!parametersHandler->getParameter("name", name))
    {
        return false;
    }

    this->setName(name.c_str());

    if (!parametersHandler->getParameter("sampling_time", m_dT))
        return false;

    if (!this->createPolydriver(parametersHandler))
    {
        std::cerr << "[Module::configure] Unable to create the polydriver." << std::endl;
        return false;
    }

    if (!this->initializeRobotControl(parametersHandler))
    {
        std::cerr << "[Module::configure] Unable to initialize the robotControl interface." << std::endl;
        return false;
    }

    if (!this->instantiateSensorBridge(parametersHandler))
    {
        std::cerr << "[Module::configure] Unable toinitialize the sensorBridge interface." << std::endl;
        return false;
    }

    m_numOfJoints = m_robotControl.getJointList().size();
    if (m_numOfJoints == 0)
    {
        std::cerr << "[Module::configure] No joints to control." << std::endl;
        return false;
    }

    m_jointNamesList = m_robotControl.getJointList();

    if(!setRobotModel(rf))
    {
        std::cerr << "[Module::configure] Unable to set the robot model.";
        return false;
    }

    if (!m_sensorBridge.advance())
    {
        std::cerr << "[Module::configure] Unable to get the robot state." << std::endl;
        return false;
    }


    //// Initialize current robot state
    m_currentJointPos.resize(m_numOfJoints);
    m_currentJointVel.resize(m_numOfJoints);
    m_currentJointVel.setZero();
    m_currentJointAcc.resize(m_numOfJoints);
    m_currentJointAcc.setZero();
    m_desJointTorque.resize(m_numOfJoints);
    m_desJointPos.resize(m_numOfJoints);
    m_desJointVel.resize(m_numOfJoints);
    m_desJointAcc.resize(m_numOfJoints);
    m_currentJointTrq.resize(m_numOfJoints);
    m_initialJointPos.resize(m_numOfJoints);
    m_initialJointPos.setZero();


    if (!parametersHandler->getParameter("initial_joint_position", m_initialJointPos))
    {
        std::cerr << "[Module::createFixedBaseTSID] initial_joint_position parameter not found.";
        return false;
    }

    // Before creating the TSID object and initialize it,
    // move the robot to the initial configuration
//    std::cout << "---------------------------------------------------------" << m_initialJointPos << std::endl;
//    m_robotControl.setReferences(m_initialJointPos,
//                                 RobotInterface::IRobotControl::ControlMode::Position);

//    if (!m_robotControl.checkMotionDone(isMotionDone, isTimeExpired, jointlist))
//    {
//        std::cerr << "[Module::updateModule] Impossible to check if the motion is done."
//                  << std::endl;
//        return false;
//    }

    if (!m_sensorBridge.getJointPositions(m_currentJointPos))
    {
        std::cerr << "[Module::configure] Sono false." << std::endl;
        return false;
    }
    m_sensorBridge.getJointVelocities(m_currentJointVel);
    m_sensorBridge.getJointAccelerations(m_currentJointAcc);
    m_sensorBridge.getJointTorques(m_currentJointTrq);

    m_kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    m_kinDyn->loadRobotModel(m_loader.model());
    m_kinDyn->setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);


    m_gravity << 0, 0, -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    m_kinDyn->setRobotState(manif::SE3d::Identity().transform(),
                            m_currentJointPos,
                            iDynTree::make_span(manif::SE3d::Tangent::Zero().data(), manif::SE3d::Tangent::DoF),
                            m_currentJointVel,
                            m_gravity);



    // create the TSID
    if (!createFixedBaseTSID(parametersHandler->getGroup("TSID").lock()))
    {
        std::cerr << "[Module::configure] Unable to create the TSID" << std::endl;
        return false;
    }

    if (!m_tsidAndTasks.regularizationTask->setSetPoint(m_currentJointPos))
    {
        std::cerr << "[Module::configure] Unable to set setpoint for regularization task." << std::endl;
        return false;
    }

    //// Create and configure the planner
    parametersHandler->getGroup("TSID").lock()->getGroup("EE_SE3_TASK").lock()->getParameter("frame_name",m_controlledFrame);
    manif::SE3d transform(BipedalLocomotion::Conversions::toManifPose(
            m_kinDyn->getWorldTransform(m_controlledFrame)));

    Eigen::Vector3d position = transform.translation();


    // first foot position
    if (!m_contactList.addContact(transform, 0.0, 2.0))
    {
        std::cerr << "[Module::configure] Unable to set desired contact." << std::endl;
        return false;
    }
    // second foot position
    position(0) -= 0.1;
    transform.translation(position);
    if (!m_contactList.addContact(transform, 5.0, 7.0))
    {
        std::cerr << "[Module::configure] Unable to set desired contact." << std::endl;
        return false;
    }
    // third foot position
    position(0) += 0.1;
    transform.translation(position);
    if (!m_contactList.addContact(transform, 10.0, 12.0))
    {
        std::cerr << "[Module::configure] Unable to set desired contact." << std::endl;
        return false;
    }
//    // fourth foot position
//    position(0) -= 0.05;
//    transform.translation(position);
//    if (!m_contactList.addContact(transform, 9.0, 10.0))
//    {
//        std::cerr << "[Module::configure] Unable to set desired contact." << std::endl;
//        return false;
//    }
//    // fifth foot position
//    position(0) += 0.05;
//    transform.translation(position);
//    if (!m_contactList.addContact(transform, 12.0, 13.0))
//    {
//        std::cerr << "[Module::configure] Unable to set desired contact." << std::endl;
//        return false;
//    }

    if (!m_planner.initialize(parametersHandler->getGroup("PLANNER")))
    {
        std::cerr << "[Module::configure] Impossible to initialize the planner." << std::endl;
        return false;
    }

    m_planner.setContactList(m_contactList);

    // Double integrator to compute desired joint positions from desired joint accelerations
    Eigen::MatrixXd A(m_numOfJoints*2,m_numOfJoints*2);
    A << Eigen::MatrixXd::Zero(m_numOfJoints,m_numOfJoints),
         Eigen::MatrixXd::Identity(m_numOfJoints,m_numOfJoints),
         Eigen::MatrixXd::Zero(m_numOfJoints,m_numOfJoints*2);

    Eigen::MatrixXd b(m_numOfJoints*2,m_numOfJoints);
    b << Eigen::MatrixXd::Zero(m_numOfJoints,m_numOfJoints),
         Eigen::MatrixXd::Identity(m_numOfJoints,m_numOfJoints);

    //std::cout << A << std::endl;
    //std::cout << b << std::endl;

    m_accSystem.dynamics = std::make_shared<LinearTimeInvariantSystem>();
    m_accSystem.dynamics->setSystemMatrices(A, b);

    Eigen::VectorXd x0(m_numOfJoints*2);
    x0 << m_currentJointPos, Eigen::MatrixXd::Zero(m_numOfJoints,1);

    m_accSystem.dynamics->setState({x0});
    //std::cout << x0 << std::endl;

    //std::cout << "Sampling time = " << m_dT << std::endl;

    m_accSystem.integrator = std::make_shared<ForwardEuler<LinearTimeInvariantSystem>>();
    m_accSystem.integrator->setIntegrationStep(m_dT);
    m_accSystem.integrator->setDynamicalSystem(m_accSystem.dynamics);

    portLog.open("/FixedBaseTSID");

    return true;
}

void Module::logData()
{
    auto & data = portLog.prepare();
    data.vectors.clear();

    data.vectors["joints_state::positions::measured"].assign(m_currentJointPos.data(), m_currentJointPos.data() + m_currentJointPos.size());
    data.vectors["joints_state::velocities::measured"].assign(m_currentJointVel.data(), m_currentJointVel.data() + m_currentJointVel.size());
    data.vectors["joints_state::accelerations::measured"].assign(m_currentJointAcc.data(), m_currentJointAcc.data() + m_currentJointAcc.size());
    data.vectors["joints_state::torques::measured"].assign(m_currentJointTrq.data(), m_currentJointTrq.data() + m_currentJointTrq.size());

    data.vectors["joints_state::positions::desired"].assign(m_desJointPos.data(), m_desJointPos.data() + m_desJointPos.size());
    data.vectors["joints_state::velocities::desired"].assign(m_desJointVel.data(), m_desJointVel.data() + m_desJointVel.size());
    data.vectors["joints_state::accelerations::desired"].assign(m_desJointAcc.data(), m_desJointAcc.data() + m_desJointAcc.size());
    data.vectors["joints_state::torques::desired"].assign(m_desJointTorque.data(), m_desJointTorque.data() + m_desJointTorque.size());

    Eigen::VectorXd generalizedBiasForces;
    generalizedBiasForces.resize(m_kinDyn->getNrOfDegreesOfFreedom() + 6);
    m_kinDyn->generalizedBiasForces(generalizedBiasForces);
    data.vectors["dynamics::bias"].assign(generalizedBiasForces.data(), generalizedBiasForces.data() + generalizedBiasForces.size());

    const manif::SE3d endEffectorPose  = Conversions::toManifPose(m_kinDyn->getWorldTransform(m_controlledFrame));
    data.vectors["cartesian::position::measured"].assign(endEffectorPose.translation().data(),
                                                         endEffectorPose.translation().data() + endEffectorPose.translation().size());
    data.vectors["cartesian::orientation::measured"].assign(endEffectorPose.quat().coeffs().data(),
                                                            endEffectorPose.quat().coeffs().data() + endEffectorPose.quat().coeffs().size());

    data.vectors["cartesian::position::desired"].assign(m_planner.getOutput().transform.translation().data(),
                                                        m_planner.getOutput().transform.translation().data()
                                                        + m_planner.getOutput().transform.translation().size());
    data.vectors["cartesian::orientation::desired"].assign(m_planner.getOutput().transform.quat().coeffs().data(),
                                                           m_planner.getOutput().transform.quat().coeffs().data()
                                                           + m_planner.getOutput().transform.quat().coeffs().size());

    portLog.write();
}

bool Module::updateModule()
{
    if (!m_sensorBridge.advance())
    {
        std::cerr << "[Module::updateModule] Unable to read the sensor." << std::endl;
        return false;
    }

    if (!m_sensorBridge.getJointPositions(m_currentJointPos))
    {
        std::cerr << "[Module::updateModule] Error in reading current joint position." << std::endl;
        return false;
    }

    if (!m_sensorBridge.getJointVelocities(m_currentJointVel))
    {
        std::cerr << "[Module::updateModule] Error in reading current joint velocity." << std::endl;
        return false;
    }

    if (!m_sensorBridge.getJointAccelerations(m_currentJointAcc))
    {
        std::cerr << "[Module::updateModule] Error in reading current joint acceleration." << std::endl;
        return false;
    }

    if (!m_sensorBridge.getJointTorques(m_currentJointTrq))
    {
        std::cerr << "[Module::updateModule] Error in reading current joint torque." << std::endl;
        return false;
    }

    if (!m_kinDyn->setRobotState(m_currentJointPos, m_currentJointVel, m_gravity))
    {
        std::cerr << "[Module::updateModule] Unable to set the robot state in kinDyn object.";
        return false;
    }

    if (!m_tsidAndTasks.se3Task->setSetPoint(m_planner.getOutput().transform,
                                        m_planner.getOutput().mixedVelocity,
                                        m_planner.getOutput().mixedAcceleration))
    {
        std::cerr << "[Module::updateModule] Unable to set setpoint for end-effector task.";
        return false;
    }

    if (!m_tsidAndTasks.tsid->advance())
    {
        std::cerr << "[Module::updateModule] Unable to update tsidAndTasks object.";
        return false;
    }

    // get the output of the TSID
    m_desJointTorque = m_tsidAndTasks.tsid->getOutput().jointTorques;
    if (!m_robotControl.setReferences(m_desJointTorque,
                                      BipedalLocomotion::RobotInterface::IRobotControl::
                                          ControlMode::Torque))
    {
        std::cerr << "[Module::updateModule] Unable to set joint torques.";
        return false;
    }

    logData();

    if (!m_planner.advance())
    {
        std::cerr << "[Module::updateModule] Unable to compute next desired pose.";
        return false;
    }

    m_currentEEPos = Conversions::toManifPose(m_kinDyn->getWorldTransform(m_controlledFrame));

    m_time += m_dT;

    return true;
}

bool Module::close()
{
    // switch back in position control
    m_robotControl.setReferences(m_currentJointPos,
                                 RobotInterface::IRobotControl::ControlMode::Position);

    return true;
}

