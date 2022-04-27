function dataset = compute_expected_contact_wrenches(config, dataset, fts)

position = dataset.q;
% velocity = dataset.dq;
% acceleration = dataset.ddq;
velocity = 0*dataset.dq;
acceleration = 0*dataset.ddq;

%    % velocity and acceleration to 0 to prove if they are neglegible. (slow
%    % experiment scenario)

dofs = length(dataset.joint_names);

q_idyn   = iDynTree.JointPosDoubleArray(dofs);
dq_idyn  = iDynTree.JointDOFsDoubleArray(dofs);
ddq_idyn = iDynTree.JointDOFsDoubleArray(dofs);

q_idyn.fromMatlab(position(1,:));
dq_idyn.fromMatlab(velocity(1,:));
ddq_idyn.fromMatlab(acceleration(1,:));


% Load the estimator and model information
estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator();


consideredJoints = iDynTree.StringVector();

for i = 1 : dofs
    consideredJoints.push_back(dataset.joint_names{i});
end

for i = 1 : size(config.ft_names_urdf,1)
    consideredJoints.push_back(config.ft_names_urdf{i});
end

estimatorLoader = iDynTree.ModelLoader();
estimatorLoader.loadReducedModelFromFile(strcat('./robots/',config.robot,'.urdf'),consideredJoints);
estimator.setModelAndSensors(estimatorLoader.model(),estimatorLoader.sensors);


% Prepare estimator variables
% Store number of sensors
n_fts = estimator.sensors().getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE);


% Gravity vector
grav_idyn = iDynTree.Vector3();
grav = [0.0;0.0;-9.81];

if strcmp(config.contact_frames,'root_link') ~= 1
    kinDyn = iDynTree.KinDynComputations();
    kinDyn.loadRobotModel(estimator.model());
    q0_idyn   = iDynTree.JointPosDoubleArray(dofs);
    q0_idyn.fromMatlab(dataset.q(1,:));
    kinDyn.setJointPos(q0_idyn);
    B_R_S = kinDyn.getRelativeTransform('root_link', 'l_sole').getRotation().toMatlab();
    grav_B = B_R_S*grav;
    grav_idyn.fromMatlab(grav_B);
else
    grav_idyn.fromMatlab(grav);
end


root_link_index = estimator.model().getFrameIndex(char('root_link'));

estimator.model().toString();

% Get ft names from urdf to know the corresponding order
ft_names_from_urdf= cell(n_fts,1);
for i = 1 : n_fts
    ft_names_from_urdf{i} = estimator.sensors().getSensor(iDynTree.SIX_AXIS_FORCE_TORQUE,i-1).getName();
end


% Set the contact information in the estimator

% Specify unknown wrenches
unknownWrench = iDynTree.UnknownWrenchContact();
unknownWrench.unknownType = iDynTree.FULL_WRENCH;
unknownWrench.contactPoint.zero();

fullBodyUnknowns = iDynTree.LinkUnknownWrenchContacts(estimator.model());
fullBodyUnknowns.clear();

contact_frame_idx = -1*ones(length(config.contact_frames),1);

for j = 1 : length(config.contact_frames)
    contact_frame_idx(j) = estimator.model().getFrameIndex(config.contact_frames{j});
    
    fullBodyUnknowns.addNewContactInFrame(estimator.model(),contact_frame_idx(j),unknownWrench);
end

% We need a new set of unknowns, as we now need 7 unknown wrenches, one for
% each submodel in the estimator
fullBodyUnknownsExtWrenchEst = iDynTree.LinkUnknownWrenchContacts(estimator.model());

estimator.updateKinematicsFromFixedBase(q_idyn,dq_idyn,ddq_idyn,root_link_index,grav_idyn);


estContactForcesExtWrenchesEst = iDynTree.LinkContactWrenches(estimator.model());

estJointTorquesExtWrenchesEst = iDynTree.JointDOFsDoubleArray(dofs);

estFTmeasurements = iDynTree.SensorsMeasurements(estimator.sensors());

ft_wrench_idyn = iDynTree.Wrench();

ft_indeces = zeros(size(config.ft_names_urdf,1),1);
for j = 1 : size(config.ft_names_urdf,1)
    ft_indeces(j) = estimator.sensors().getSensorIndex(iDynTree.SIX_AXIS_FORCE_TORQUE,config.ft_names_urdf{j});
end

len = size(dataset.q,1);
len = 1000;

% For each timestamp instant

displayed = 0;

for i = 1 : len
    
    q_idyn.fromMatlab(position(i,:));
    dq_idyn.fromMatlab(velocity(i,:));
    ddq_idyn.fromMatlab(acceleration(i,:));
    
    for j = 1 : length(config.ft_names_urdf)
        ft_wrench_idyn.fromMatlab(fts.(config.ft_names_urdf{j})(i,:));
        estFTmeasurements.setMeasurement(iDynTree.SIX_AXIS_FORCE_TORQUE,ft_indeces(j),ft_wrench_idyn);
    end
    
    estimator.updateKinematicsFromFixedBase(q_idyn,dq_idyn,ddq_idyn,root_link_index,grav_idyn);
    
    fullBodyUnknownsExtWrenchEst.clear();
    
    for j = 1 : length(config.ft_ext_wrench_frames)
        fullBodyUnknownsExtWrenchEst.addNewUnknownFullWrenchInFrameOrigin(estimator.model(),estimator.model().getFrameIndex(config.ft_ext_wrench_frames{j}));
    end
    
    estimator.estimateExtWrenchesAndJointTorques(fullBodyUnknownsExtWrenchEst,estFTmeasurements,estContactForcesExtWrenchesEst,estJointTorquesExtWrenchesEst);
    
    dataset.estimatedJointTorques(i,:) = estJointTorquesExtWrenchesEst.toMatlab()';
    
    linkNetExtWrenches = iDynTree.LinkWrenches(estimator.model());
    
    estContactForcesExtWrenchesEst.computeNetWrenches(linkNetExtWrenches);
    
    for j = 1 : length(config.ft_ext_wrench_frames)
        contact_wrench_idyn = linkNetExtWrenches(estimator.model().getFrameLink(estimator.model().getFrameIndex(config.ft_ext_wrench_frames{j})));
        dataset.tested_external_wrenches.(config.ft_ext_wrench_frames{j})(i,:) = contact_wrench_idyn.toMatlab();
    end
    
    if rem(i*100/len,10) == 0
        if ~displayed
            displayed = 1;
            disp(strcat('Completed --> ', {' '}, num2str(floor(i*100/len)), ' %'));
        end
    else
        displayed = 0;
    end
    
end

end

