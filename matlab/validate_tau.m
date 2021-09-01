addpath('KF');
addpath('URDF');

% list_of_joints = {'hippitch','hiproll','hipyaw','knee','anklepitch','ankleroll'};

% Select the joint to consider
% list = {'hippitch','hiproll','hipyaw',...                    
% 'knee','anklepitch','ankleroll'};
% [joint_from_the_list,tf] = listdlg('ListString',list);

gearbox_values = [100.00  -100.00   100.00   100.00  -100.00   -100.00];

select_dataset = false;
if (select_dataset)
  datasetStruct = load_data(select_dataset);
end

tau_joint = datasetStruct{1, 1}.Joint_state.joint_torques;
s_dataset = datasetStruct{1, 1}.Joint_state.joint_positions;
sdot_dataset = datasetStruct{1, 1}.Joint_state.joint_velocities;

[sdot_dataset,sddot_dataset] = estimate_joints_vel_acc(sdot_dataset);

% Save the position of the input urdf
input_urdf = 'URDF/iCub2_right_leg.urdf';
% The model calibration helper is a loader for URDF files
mdlLoader = iDynTree.ModelCalibrationHelper();

% input urdf file to acquire robot structure
mdlLoader.loadModelFromFile(input_urdf);

kinDynComp = iDynTree.KinDynComputations();
kinDynComp.loadRobotModel(mdlLoader.model());
nrOfDOFs = kinDynComp.model().getNrOfDOFs();

grav = iDynTree.Vector3();
grav.zero();
grav.setVal(2, -9.80665);

base_acc = iDynTree.Vector6;
base_acc.zero();

kinDyn_joint_torques = zeros(size(tau_joint));

s_idyn = iDynTree.VectorDynSize(nrOfDOFs);
ds_idyn = iDynTree.VectorDynSize(nrOfDOFs);
dds_idyn = iDynTree.VectorDynSize(nrOfDOFs);

linkNetExtWrenches = iDynTree.LinkWrenches(kinDynComp.model());
estContactForcesExtWrenchesEst = iDynTree.LinkContactWrenches(kinDynComp.model());
estContactForcesExtWrenchesEst.computeNetWrenches(linkNetExtWrenches);
tau_offline_sample = iDynTree.FreeFloatingGeneralizedTorques(kinDynComp.model());

for sample = 1:size(tau_joint,2)
    % Read q at time instant
    s = s_dataset(:,sample);
    ds = sdot_dataset(:,sample);
    dds = sddot_dataset(:,sample);

    s_idyn.fromMatlab(s); 
    ds_idyn.fromMatlab(ds); 
    dds_idyn.fromMatlab(dds);

    kinDynComp.setRobotState(s_idyn, ds_idyn, grav);
    
    kinDynComp.inverseDynamics(base_acc,dds_idyn,linkNetExtWrenches,tau_offline_sample);
        
    joint_torques = tau_offline_sample.jointTorques;
  
    kinDyn_joint_torques(:,sample) = joint_torques.toMatlab()';
end

figure,
plot(kinDyn_joint_torques')
hold on
plot(tau_joint')
title('iDynTree_offline vs iDynTree_online')
