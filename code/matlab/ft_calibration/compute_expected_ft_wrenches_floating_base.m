function dataset = compute_expected_ft_wrenches_floating_base(config, dataset, estimator)

%    % velocity and acceleration to 0 to prove if they are neglegible. (slow
%    % experiment scenario)

dofs = length(dataset.joint_names);

q_idyn   = iDynTree.JointPosDoubleArray(dofs);
dq_idyn  = iDynTree.JointDOFsDoubleArray(dofs);
ddq_idyn = iDynTree.JointDOFsDoubleArray(dofs);

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

% Prepare estimator variables
% Store number of sensors
n_fts = estimator.sensors().getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE);

% The estimated FT sensor measurements
expFTmeasurements = iDynTree.SensorsMeasurements(estimator.sensors());

% The estimated external wrenches
estContactForces = iDynTree.LinkContactWrenches(estimator.model());

% The estimated joint torques
estJointTorques = iDynTree.JointDOFsDoubleArray(dofs);

% Set the contact information in the estimator
contact_index = -1*ones(length(config.contact_frames),1);
for j = 1 : length(config.contact_frames)
    contact_index(j) = estimator.model().getFrameIndex(char(config.contact_frames{j}));
end

root_link_index = estimator.model().getFrameIndex(char('root_link'));

estimator.model().toString();


% Get ft names from urdf to know the corresponding order
ft_names_from_urdf= cell(n_fts,1);
for i = 1 : n_fts
    ft_names_from_urdf{i} = estimator.sensors().getSensor(iDynTree.SIX_AXIS_FORCE_TORQUE,i-1).getName();
end


len = size(dataset.q,1);

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

progress_bar('Computing expected ft values: ');

for i = 1 : len
    
    q   = dataset.q(i,:);
    
    if config.use_velocity_and_acceleration
        dq  = dataset.dq(i,:);
        ddq = dataset.ddq(i,:);
    else
        dq  = 0*dataset.dq(i,:);
        ddq = 0*dataset.ddq(i,:);
    end
    
    q_idyn.fromMatlab(q);
    dq_idyn.fromMatlab(dq);
    ddq_idyn.fromMatlab(ddq);
    
    % Update robot kinematics
    estimator.updateKinematicsFromFixedBase(q_idyn,dq_idyn,ddq_idyn,root_link_index,grav_idyn);
    
    fullBodyUnknownsExtWrenchEst.clear();
    
    for j = 1 : length(config.ft_ext_wrench_frames)
        fullBodyUnknownsExtWrenchEst.addNewUnknownFullWrenchInFrameOrigin(estimator.model(),estimator.model().getFrameIndex(config.ft_ext_wrench_frames{j}));
    end
    
    % Sensor wrench buffer
    estimatedSensorWrench = iDynTree.Wrench();
    estimatedSensorWrench.fromMatlab(zeros(1,6));
    
    % Run the prediction of FT measurements
    
    % run the estimation
    estimator.computeExpectedFTSensorsMeasurements(fullBodyUnknowns,expFTmeasurements,estContactForces,estJointTorques);
    
    % store the estimated measurements
    for j = 0:(n_fts-1)
        expFTmeasurements.getMeasurement(iDynTree.SIX_AXIS_FORCE_TORQUE, j, estimatedSensorWrench);
        
        expected_fts.(ft_names_from_urdf{j+1})(i,:) = estimatedSensorWrench.toMatlab()';
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    linkNetExtWrenches = iDynTree.LinkWrenches(estimator.model());
    
    estContactForces.computeNetWrenches(linkNetExtWrenches);
    
    for j = 1 : length(config.ft_ext_wrench_frames)
        contact_wrench_idyn = linkNetExtWrenches(estimator.model().getFrameLink(estimator.model().getFrameIndex(config.ft_ext_wrench_frames{j})));
        dataset.estimated_external_wrenches.(config.ft_ext_wrench_frames{j})(i,:) = contact_wrench_idyn.toMatlab();
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %     for j = 1 : length(config.ft_ext_wrench_frames)
    %         dataset.external_wrenches.(config.ft_ext_wrench_frames{j})(i,:) = ...
    %             estContactForces.contactWrench(estimator.model().getLinkIndex(config.ft_ext_wrench_frames{j}),0).contactWrench().getLinearVec3().toMatlab();
    %     end
    
    progress_bar(i*100/len);
end

dataset.expected_fts = expected_fts;

save(strcat('./data/parsed/',config.experiments{1},'_parsed.mat'),'dataset');

end

