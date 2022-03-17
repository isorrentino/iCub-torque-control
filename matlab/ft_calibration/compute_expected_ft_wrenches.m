function dataset = compute_expected_ft_wrenches(config, dataset, estimator)

% Gravity vector
grav_idyn = iDynTree.Vector3();
grav = [0.0;0.0;-9.81];
grav_idyn.fromMatlab(grav);

%    % velocity and acceleration to 0 to prove if they are neglegible. (slow
%    % experiment scenario)

dofs = length(dataset.joint_names);

q_idyn   = iDynTree.JointPosDoubleArray(dofs);
dq_idyn  = iDynTree.JointDOFsDoubleArray(dofs);
ddq_idyn = iDynTree.JointDOFsDoubleArray(dofs);

% Prepare estimator variables
% Store number of sensors
n_fts = estimator.sensors().getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE);

% The estimated FT sensor measurements
estFTmeasurements = iDynTree.SensorsMeasurements(estimator.sensors());

% The estimated external wrenches
estContactForces = iDynTree.LinkContactWrenches(estimator.model());

% The estimated joint torques
estJointTorques = iDynTree.JointDOFsDoubleArray(dofs);

% Set the contact information in the estimator
contact_index = estimator.model().getFrameIndex(char(config.contact_link));

estimator.model().toString();


% Get ft names from urdf to know the corresponding order
ft_names_from_urdf= cell(n_fts,1);
for i = 1 : n_fts
    ft_names_from_urdf{i} = estimator.sensors().getSensor(iDynTree.SIX_AXIS_FORCE_TORQUE,i-1).getName();
end


len = size(dataset.q,1);

progress_bar('Computing expected ft values: ');

for i = 1 : len
    
    q   = dataset.q(i,:);
    dq  = dataset.dq(i,:);
    ddq = dataset.ddq(i,:);
    
    q_idyn.fromMatlab(q);
    dq_idyn.fromMatlab(dq);
    ddq_idyn.fromMatlab(ddq);
    
    % Update robot kinematics
    estimator.updateKinematicsFromFixedBase(q_idyn,dq_idyn,ddq_idyn,contact_index,grav_idyn);
    
    % Specify unknown wrenches
    unknownWrench = iDynTree.UnknownWrenchContact();
    
    
    % Run the prediction of FT measurements
    
    % There are three output of the estimation, FT measurements, contact
    % forces and joint torques
    fullBodyUnknowns = iDynTree.LinkUnknownWrenchContacts(estimator.model());
    fullBodyUnknowns.clear();
    unknownWrench.unknownType = iDynTree.FULL_WRENCH;
    unknownWrench.contactPoint = iDynTree.Position.Zero();
    fullBodyUnknowns.addNewContactInFrame(estimator.model(),contact_index,unknownWrench);
    
    
    % Sensor wrench buffer
    estimatedSensorWrench = iDynTree.Wrench();
    estimatedSensorWrench.fromMatlab(zeros(1,6));
    
    
    % run the estimation
    estimator.computeExpectedFTSensorsMeasurements(fullBodyUnknowns,estFTmeasurements,estContactForces,estJointTorques);
    
    
    % store the estimated measurements
    for j = 0:(n_fts-1)
        estFTmeasurements.getMeasurement(iDynTree.SIX_AXIS_FORCE_TORQUE, j, estimatedSensorWrench);
        
        expected_fts.(ft_names_from_urdf{j+1})(i,:) = estimatedSensorWrench.toMatlab()';
    end
    
    progress_bar(i*100/len);
end

dataset.expected_fts = expected_fts;

save(strcat('./data/parsed/',config.experiment,'_parsed.mat'),'dataset');

end

