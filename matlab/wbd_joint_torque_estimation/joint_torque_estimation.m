addpath(genpath('../'));

%% Load dataset

load_dataset_bool = true;

if load_dataset_bool
    clear;
    close all;
    num_datasets = 1;
    load_dataset_bool = true;
end

use_joint_acceleration = false;

new_logger_dataset = false;

joint = 21:26;

if new_logger_dataset
    load_dataset_new;
else
    load_dataset_old;
end

% Estimate joint acceleration **pure kinematic KF filter**
[~, joint_acc] = estimateVelAccFromPos(joint_pos, timestamp(2)-timestamp(1));


%% Load model and estimator

icubModelsInstallPrefix = getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX');
robotName='iCubGenova09';

modelPath = [icubModelsInstallPrefix '/share/iCub/robots/' robotName '/'];
fileName='model.urdf';

consideredJoints = {'r_hip_pitch';
                    'r_hip_roll';
                    'r_hip_yaw';
                    'r_knee';
                    'r_ankle_pitch';
                    'r_ankle_roll'};

% Get joint information
dofs = length(consideredJoints);

consideredFT = {'r_foot_front_ft_sensor';'r_foot_rear_ft_sensor'};

nrOfFTSensors = 2;

dofs = length(consideredJoints);
consideredJointsiDyn = iDynTree.StringVector();
for i=1:dofs
    consideredJointsiDyn.push_back((consideredJoints{i}));
end
for i=1:length(consideredFT)
    consideredJointsiDyn.push_back((consideredFT{i}));
end


estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator();

% Load model and sensors from the URDF file
estimatorLoader = iDynTree.ModelLoader();
estimatorLoader.loadReducedModelFromFile(strcat(modelPath,fileName),consideredJointsiDyn);
estimator.setModelAndSensors(estimatorLoader.model(),estimatorLoader.sensors);

estimator.model().toString() % print model


%% Set kinematic information

% Gravity vector
grav_idyn = iDynTree.Vector3();
grav = [0.0;0.0;-9.81];
grav_idyn.fromMatlab(grav);

% Set contact information
contact_frame_index = estimator.model().getFrameIndex('root_link');

% We first need a new set of unknown wrenches acting at the
% root_link and right foot (rear and front links).
fullBodyUnknownsExtWrenchEst = iDynTree.LinkUnknownWrenchContacts(estimator.model());

frameNames = {'root_link','r_foot_front','r_foot_rear'};
for frame=1:length(frameNames)
    fullBodyUnknownsExtWrenchEst.addNewUnknownFullWrenchInFrameOrigin(estimator.model(),estimator.model().getFrameIndex(frameNames{frame}));
end

% Joint state as iDyntree vectors
qj_idyn   = iDynTree.JointPosDoubleArray(dofs);
dqj_idyn  = iDynTree.JointDOFsDoubleArray(dofs);
ddqj_idyn = iDynTree.JointDOFsDoubleArray(dofs);

% We also need to allocate the output
% The estimated FT sensor measurements
estFTmeasurements = iDynTree.SensorsMeasurements(estimator.sensors());

% Eexternal wrenches
estContactForcesExtWrenchesEst = iDynTree.LinkContactWrenches(estimator.model());

% Joint torques
estJointTorquesExtWrenchesEst = iDynTree.JointDOFsDoubleArray(dofs);

%size of arrays with the expected Data
externalWrenchData = zeros(length(frameNames),length(timestamp),6);

estimatedJointTorques = zeros(size(joint_pos));

wrench_idyn= iDynTree.Wrench();


%% For each timestamp instant
disp('Computing the joint torques and external wrenches');

for sample = 1 : length(timestamp)
    qj_idyn.fromMatlab(joint_pos(:,sample)');
    dqj_idyn.fromMatlab(joint_vel(:,sample)');
    
    if use_joint_acceleration == true
        ddqj_idyn.fromMatlab(joint_acc(:,sample)');
    else
        ddqj_idyn.fromMatlab(zeros(6,1)');
    end
    
    wrench_idyn.fromMatlab(right_front_wrench_client(:,sample));
    estFTmeasurements.setMeasurement(iDynTree.SIX_AXIS_FORCE_TORQUE,0,wrench_idyn);
    
    wrench_idyn.fromMatlab(right_rear_wrench_client(:,sample));
    estFTmeasurements.setMeasurement(iDynTree.SIX_AXIS_FORCE_TORQUE,1,wrench_idyn);
    
    estimator.updateKinematicsFromFixedBase(qj_idyn,dqj_idyn,ddqj_idyn,contact_frame_index,grav_idyn);
    
    estimator.estimateExtWrenchesAndJointTorques(fullBodyUnknownsExtWrenchEst,estFTmeasurements,estContactForcesExtWrenchesEst,estJointTorquesExtWrenchesEst);
    
    estimatedJointTorques(:,sample) = estJointTorquesExtWrenchesEst.toMatlab();
    
    linkNetExtWrenches = iDynTree.LinkWrenches(estimator.model());%
    estContactForcesExtWrenchesEst.computeNetWrenches(linkNetExtWrenches);
    for i=1:length(frameNames)
        wrench = linkNetExtWrenches(estimator.model().getFrameLink(estimator.model().getFrameIndex(frameNames{i})));
        externalWrenchData(i,sample,:) = wrench.toMatlab();
    end
end


%% Plot measured joint torques VS estimated joint torques

% Load desired torques
[file,path] = uigetfile('*.mat','Select dataset for identification');
matToLoad = [path, file];
load(matToLoad);

joint_trq_des(1,:) = r_hip_pitch_destrq;
joint_trq_des(2,:) = r_hip_roll_destrq;
joint_trq_des(3,:) = r_hip_yaw_destrq;
joint_trq_des(4,:) = r_knee_destrq;
joint_trq_des(5,:) = r_ankle_pitch_destrq;
joint_trq_des(6,:) = r_ankle_roll_destrq;

figure
for i = 1 : 6
    subplot(2,3,i)
    plot(time,joint_trq_des(i,:))
    hold on
    plot(timestamp,joint_trq(i,:))
    plot(timestamp,estimatedJointTorques(i,:))
    xlabel('timestamp (sec)')
    ylabel('\tau')
    title(consideredJoints{i},'Interpreter','None')
    legend('Desired','Measured','Estimated')
end


figure
for i = 1 : 6
    subplot(2,3,i)
    plot(timestamp-timestamp(1),right_front_wrench_client(i,:))
    hold on
    plot(timestamp-timestamp(1),reshape(externalWrenchData(2,:,i),size(externalWrenchData,2),1))
    xlabel('timestamp (sec)')
    ylabel('f_{ext}')
    title(frameNames{2},'Interpreter','None')
    legend('Measured','Estimated')
end


figure
for i = 1 : 6
    subplot(2,3,i)
    plot(timestamp-timestamp(1),right_rear_wrench_client(i,:))
    hold on
    plot(timestamp-timestamp(1),reshape(externalWrenchData(3,:,i),size(externalWrenchData,2),1))
    xlabel('timestamp (sec)')
    ylabel('f_{ext}')
    title(frameNames{3},'Interpreter','None')
    legend('Measured','Estimated')
end
