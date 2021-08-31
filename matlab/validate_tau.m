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

tau_joint = datasetStruct{1, 1}.Joint_state.joint_torques';
s = datasetStruct{1, 1}.Joint_state.joint_positions';
sdot = datasetStruct{1, 1}.Joint_state.joint_velocities';

% Estimate acceleration
stateSize = 3;
sddot = zeros(size(sdot));
for j = 1 : 6
  x0 = [s(1,j); zeros(stateSize-1,1)];
  kf = initKF(x0, stateSize);
  % Run filter for entire traj
  xh = zeros(stateSize, size(s,1));
  for sample = 1 : size(s,1)
    kf.z = s(sample,j);
    kf = kalmanfilter(kf);
    xh(:,sample) = kf.x;
  end
  sdot(:,j) = xh(2,:)';
  sddot(:,j) = xh(3,:)';
end

% Save the position of the input urdf
input_urdf = 'URDF/iCub2_right_leg.urdf';
% The model calibration helper is a loader for URDF files
mdlLoader = iDynTree.ModelCalibrationHelper();

% input urdf file to acquire robot structure
mdlLoader.loadModelFromFile(input_urdf);

kinDynComp = iDynTree.KinDynComputations();
kinDynComp.loadRobotModel(mdlLoader.model());
nrOfDOFs = kinDynComp.model().getNrOfDOFs();

jointPos = zeros(nrOfDOFs, 1);
jointVel = zeros(nrOfDOFs, 1);
jointAcc = zeros(nrOfDOFs, 1);
s_idyn = iDynTree.VectorDynSize(nrOfDOFs);
s_idyn.fromMatlab(jointPos);
ds_idyn = iDynTree.VectorDynSize(nrOfDOFs);
ds_idyn.fromMatlab(jointVel);
dds_idyn = iDynTree.VectorDynSize(nrOfDOFs);
dds_idyn.fromMatlab(jointAcc);

base_acc = iDynTree.Vector6;


grav = iDynTree.Vector3();
grav.zero();
grav.setVal(2, -9.80665);

kinDynComp.setRobotState(q_idyn, ds_idyn, grav);

linkNetExtWrenches = iDynTree.LinkWrenches(kinDynComp.model());
estContactForcesExtWrenchesEst = iDynTree.LinkContactWrenches(kinDynComp.model());
estContactForcesExtWrenchesEst.computeNetWrenches(linkNetExtWrenches);

tau_offline_sample = iDynTree.FreeFloatingGeneralizedTorques(kinDynComp.model());

for sample = 1 : size(s,1)
  dds_idyn.fromMatlab(sddot(sample,:));
  
  kinDynComp.inverseDynamics(base_acc,dds_idyn,linkNetExtWrenches,tau_offline_sample);

  joint_torques = tau_offline_sample.jointTorques;
  
  kinDyn_joint_torques(sample,:) = joint_torques.toMatlab()';
end
