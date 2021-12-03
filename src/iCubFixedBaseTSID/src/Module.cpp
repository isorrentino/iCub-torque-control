/**
 * @file Module.cpp
 * @authors Ines Sorrentino <ines.sorrentino@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iomanip>
#include <iCub-torque-control/iCubFixedBaseTSID/Module.h>

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
    if (!m_tsidAndTasks.tsid->addTask(m_tsidAndTasks.se3Task, "se3_task", highPriority))
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
    m_desJointTorque.resize(m_numOfJoints);
    m_desJointPos.resize(m_numOfJoints);
    m_desJointVel.resize(m_numOfJoints);
    m_desJointAcc.resize(m_numOfJoints);

    m_sensorBridge.getJointPositions(m_currentJointPos);
    m_sensorBridge.getJointVelocities(m_currentJointVel);

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
    if (!m_contactList.addContact(transform, 0.0, 1.0))
    {
        std::cerr << "[Module::configure] Unable to set desired contact." << std::endl;
        return false;
    }
    // second foot position
    position(0) -= 0.05;
    transform.translation(position);
    if (!m_contactList.addContact(transform, 3.0, 4.0))
    {
        std::cerr << "[Module::configure] Unable to set desired contact." << std::endl;
        return false;
    }
    // third foot position
    position(0) += 0.05;
    transform.translation(position);
    if (!m_contactList.addContact(transform, 6.0, 7.0))
    {
        std::cerr << "[Module::configure] Unable to set desired contact." << std::endl;
        return false;
    }
    // fourth foot position
    position(0) -= 0.05;
    transform.translation(position);
    if (!m_contactList.addContact(transform, 9.0, 10.0))
    {
        std::cerr << "[Module::configure] Unable to set desired contact." << std::endl;
        return false;
    }

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

    m_accSystem.dynamics = std::make_shared<LinearTimeInvariantSystem>();
    m_accSystem.dynamics->setSystemMatrices(A, b);

    Eigen::VectorXd x0(m_numOfJoints*2);
    x0 << m_currentJointPos, m_currentJointVel;
    m_accSystem.dynamics->setState({x0});

    m_accSystem.integrator = std::make_shared<ForwardEuler<LinearTimeInvariantSystem>>();
    m_accSystem.integrator->setIntegrationStep(m_dT);
    m_accSystem.integrator->setDynamicalSystem(m_accSystem.dynamics);

    return true;
}

void Module::logData()
{
    // log data
    m_log["time"].push_back(BipedalLocomotion::clock().now().count());

    for (int i = 0; i < m_numOfJoints; i++)
    {
        m_log[m_jointNamesList[i] + "_pos"].push_back(m_currentJointPos[i]);
    }

    for (int i = 0; i < m_numOfJoints; i++)
    {
        m_log[m_jointNamesList[i] + "_vel"].push_back(m_currentJointVel[i]);
    }

    for (int i = 0; i < m_numOfJoints; i++)
    {
        m_log[m_jointNamesList[i] + "_destrq"].push_back(m_desJointTorque[i]);
    }

    for (int i = 0; i < m_numOfJoints; i++)
    {
        m_log[m_jointNamesList[i] + "_despos"].push_back(m_desJointPos[i]);
    }

    for (int i = 0; i < m_numOfJoints; i++)
    {
        m_log[m_jointNamesList[i] + "_desvel"].push_back(m_desJointVel[i]);
    }

    for (int i = 0; i < m_numOfJoints; i++)
    {
        m_log[m_jointNamesList[i] + "_desacc"].push_back(m_desJointAcc[i]);
    }

    Eigen::VectorXd generalizedBiasForces;
    generalizedBiasForces.resize(m_kinDyn->getNrOfDegreesOfFreedom() + 6);
    m_kinDyn->generalizedBiasForces(generalizedBiasForces);
    for (int i = 0; i < m_numOfJoints; i++)
    {
        m_log[m_jointNamesList[i] + "_bias"].push_back(generalizedBiasForces[i]);
    }

    // Check the end-effector pose error
    const manif::SE3d endEffectorPose  = Conversions::toManifPose(m_kinDyn->getWorldTransform(m_controlledFrame));
    m_log["ee_x"].push_back(endEffectorPose.translation()[0]);
    m_log["ee_y"].push_back(endEffectorPose.translation()[1]);
    m_log["ee_z"].push_back(endEffectorPose.translation()[2]);
    m_log["ee_quat_x"].push_back(endEffectorPose.quat().coeffs()[0]);
    m_log["ee_quat_y"].push_back(endEffectorPose.quat().coeffs()[1]);
    m_log["ee_quat_z"].push_back(endEffectorPose.quat().coeffs()[2]);
    m_log["ee_quat_w"].push_back(endEffectorPose.quat().coeffs()[3]);

    m_log["ee_des_x"].push_back(m_planner.getOutput().transform.translation()[0]);
    m_log["ee_des_y"].push_back(m_planner.getOutput().transform.translation()[1]);
    m_log["ee_des_z"].push_back(m_planner.getOutput().transform.translation()[2]);
    m_log["ee_des_quat_x"].push_back(m_planner.getOutput().transform.quat().coeffs()[0]);
    m_log["ee_des_quat_y"].push_back(m_planner.getOutput().transform.quat().coeffs()[1]);
    m_log["ee_des_quat_z"].push_back(m_planner.getOutput().transform.quat().coeffs()[2]);
    m_log["ee_des_quat_w"].push_back(m_planner.getOutput().transform.quat().coeffs()[3]);

    m_log["ee_des_dx"].push_back(m_planner.getOutput().mixedVelocity.data()[0]);
    m_log["ee_des_dy"].push_back(m_planner.getOutput().mixedVelocity.data()[1]);
    m_log["ee_des_dz"].push_back(m_planner.getOutput().mixedVelocity.data()[2]);

    m_log["ee_des_ddx"].push_back(m_planner.getOutput().mixedAcceleration.data()[0]);
    m_log["ee_des_ddy"].push_back(m_planner.getOutput().mixedAcceleration.data()[1]);
    m_log["ee_des_ddz"].push_back(m_planner.getOutput().mixedAcceleration.data()[2]);
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
        std::cerr << "[Module::updateModule] Error in reading current position." << std::endl;
        return false;
    }

    if (!m_sensorBridge.getJointVelocities(m_currentJointVel))
    {
        std::cerr << "[Module::updateModule] Error in reading current velocity." << std::endl;
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
    m_desJointAcc = m_tsidAndTasks.tsid->getOutput().jointAccelerations;

    std::cout << "desired accelerations" << std::endl;
    std::cout << m_desJointAcc << std::endl;

    std::cout << "desired torques" << std::endl;
    std::cout << m_desJointTorque << std::endl;

    m_accSystem.dynamics->setControlInput({m_desJointAcc});
    m_accSystem.integrator->integrate(0, m_dT);
    Eigen::VectorXd solution = std::get<0>(m_accSystem.integrator->getSolution());

    m_desJointPos = solution.head(m_numOfJoints);
    m_desJointVel = solution.tail(m_numOfJoints);

    std::cout << "desired" << std::endl;
    std::cout << m_desJointPos << std::endl;

    std::cout << "current" << std::endl;
    std::cout << m_currentJointPos << std::endl;

    if (!m_robotControl.setReferences(m_desJointPos, BipedalLocomotion::RobotInterface::IRobotControl::ControlMode::PositionDirect))

    {
        std::cerr << "[Module::updateModule] Unable to set desired joint positions.";
        return false;
    }

    logData();

    if (!m_planner.advance())
    {
        std::cerr << "[Module::updateModule] Unable to compute next desired pose.";
        return false;
    }

    m_currentEEPos = Conversions::toManifPose(m_kinDyn->getWorldTransform(m_controlledFrame));

    return true;
}

bool Module::close()
{
    // switch back in position control
    m_robotControl.setReferences(m_currentJointPos,
                                 RobotInterface::IRobotControl::ControlMode::Position);

    std::cout << "[Module::close] I'm storing the dataset." << std::endl;
    // set the file name
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::stringstream fileName;

    fileName << "Dataset_Measured_" << m_robotControl.getJointList().front() << "_"
             << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S") << ".mat";

    matioCpp::File file = matioCpp::File::Create(fileName.str());

    for (auto& [key, value] : m_log)
    {
        matioCpp::Vector<double> out(key);
        out = value;
        file.write(out);
    }

    std::cout << "[Module::close] Dataset stored. Closing." << std::endl;

    return true;
}

