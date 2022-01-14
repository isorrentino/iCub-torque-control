addpath(genpath('..'));

modelPath = '../urdf/';
modelName = 'iCub3_right_leg.urdf';

num_datasets = 1;
load_dataset_bool = true;
leg = 'right';
joint = 21:26;
load_dataset;

%% Prepare data
[~, joint_acc] = estimateVelAccFromPos(joint_pos);

%% Load the estimator
estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator();

% Load model and sensors from the URDF file
estimator.loadModelAndSensorsFromFile(strcat(modelPath,modelName));
estimator.model().toString() % print model

%% Set kinematics information
grav_idyn = iDynTree.Vector3();
grav = [0.0;0.0;-9.81];
grav_idyn.fromMatlab(grav);

% Get joint information
dofs = estimator.model().getNrOfDOFs();

qj_idyn   = iDynTree.JointPosDoubleArray(dofs);
dqj_idyn  = iDynTree.JointDOFsDoubleArray(dofs);
ddqj_idyn = iDynTree.JointDOFsDoubleArray(dofs);

qj = joint_pos(:,1);
dqj = joint_vel(:,1);
ddqj = joint_acc(:,1);

qj_idyn.fromMatlab(qj);
dqj_idyn.fromMatlab(dqj);
ddqj_idyn.fromMatlab(ddqj);

% Set the kinematics information in the estimator
root_link_index = estimator.model().getFrameIndex('root_link');
estimator.updateKinematicsFromFixedBase(qj_idyn,dqj_idyn,ddqj_idyn,root_link_index,grav_idyn);

%% Specify unknown wrenches
% We need to set the location of the unknown wrench. We express the unknown
% wrench at the origin of the l_sole frame
unknownWrench = iDynTree.UnknownWrenchContact();
unknownWrench.unknownType = iDynTree.FULL_WRENCH;

%% Set contacts
framesNames={'r_foot_rear','r_foot_front','root_link'};
for frame=1:length(framesNames) 
    fullBodyUnknownsExtWrenchEst.addNewUnknownFullWrenchInFrameOrigin(estimator.model(),estimator.model().getFrameIndex(framesNames{frame}));
end

% The estimated external wrenches
estContactForcesExtWrenchesEst = iDynTree.LinkContactWrenches(estimator.model());


% The estimated joint torques
estJointTorquesExtWrenchesEst = iDynTree.JointDOFsDoubleArray(dofs);


for ftIndex = 0:(nrOfFTSensors-1)
    sens = estimator.sensors().getSensor(iDynTree.SIX_AXIS_FORCE_TORQUE,ftIndex).getName();
    matchup(ftIndex+1) = find(strcmp(sensorNames,sens ));
end


for t=1:size(joint_pos,1)
    qj=joint_pos(:,t);
    dqj=dqj_all(:,t);
    ddqj=ddqj_all(:,t);
    
    
    qj_idyn.fromMatlab(qj);
    dqj_idyn.fromMatlab(dqj);
    ddqj_idyn.fromMatlab(ddqj);
    
    if(length(contactFrameName)>1)
        contact_index = estimator.model().getFrameIndex(char(contactFrameName(t)));
    end
    
    ok = estimator.updateKinematicsFromFixedBase(qj_idyn,dqj_idyn,ddqj_idyn,contact_index,grav_idyn);
end




