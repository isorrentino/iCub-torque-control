addpath('KF');
addpath('URDF');

% list_of_joints = {'hippitch','hiproll','hipyaw','knee','anklepitch','ankleroll'};

% Select the joint to consider
% list = {'hippitch','hiproll','hipyaw',...                    
% 'knee','anklepitch','ankleroll'};
% [joint_from_the_list,tf] = listdlg('ListString',list);

gearbox_values = [100.00  -100.00   100.00   100.00  100.00   100.00];

select_dataset = true;
if (select_dataset)
  datasetStruct = load_data(select_dataset);
end

tau_joint = datasetStruct{1, 1}.Joint_state.joint_torques;
s_dataset = datasetStruct{1, 1}.Joint_state.joint_positions;
sdot_dataset = datasetStruct{1, 1}.Joint_state.joint_velocities;

[~,sddot_dataset] = estimate_joints_vel_acc(s_dataset);
% sddot_dataset = zeros(size(sdot_dataset));

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
for i = 1 : 6
subplot(2,3,i)
plot(kinDyn_joint_torques(i,:)')
hold on
plot(tau_joint(i,:)')
title(['Joint ',num2str(i)],'Interpreter','none')
legend('offline not smoothed','online smoothed')
xlabel('samples')
ylabel('\tau')
end


figure,
for i = 1 : 6
subplot(2,3,i)
plot(kinDyn_joint_torques(i,:)'-tau_joint(i,:)')
title(['Joint ',num2str(i)],'Interpreter','none')
% legend('offline','online')
xlabel('samples')
ylabel('error \tau')
end

figure,
for i = 1 : 6
subplot(2,3,i)
plot(s_dataset(i,:)'.*180./pi)
title(['Joint ',num2str(i)],'Interpreter','none')
% legend('offline','online')
xlabel('samples')
ylabel('joint pos')
end


kinDyn_joint_torques_zero_jointAcc = zeros(size(tau_joint));

for sample = 1:size(tau_joint,2)
    % Read q at time instant
    s = s_dataset(:,sample);
    ds = sdot_dataset(:,sample);
    dds = sddot_dataset(:,sample);

    s_idyn.fromMatlab(s); 
    ds_idyn.fromMatlab(ds); 
    dds_idyn.fromMatlab(zeros(6,1));

    kinDynComp.setRobotState(s_idyn, ds_idyn, grav);
    
    kinDynComp.inverseDynamics(base_acc,dds_idyn,linkNetExtWrenches,tau_offline_sample);
        
    joint_torques = tau_offline_sample.jointTorques;
  
    kinDyn_joint_torques_zero_jointAcc(:,sample) = joint_torques.toMatlab()';
end

figure,
for i = 1 : 6
subplot(2,3,i)
plot(kinDyn_joint_torques_zero_jointAcc(i,:)')
hold on
plot(tau_joint(i,:)')
title(['Joint ',num2str(i)],'Interpreter','none')
legend('offline not smoothed','online smoothed')
xlabel('samples')
ylabel('\tau')
end


figure,
for i = 1 : 6
subplot(2,3,i)
plot(kinDyn_joint_torques_zero_jointAcc(i,:)'-tau_joint(i,:)')
title(['Joint ',num2str(i)],'Interpreter','none')
% legend('offline','online')
xlabel('samples')
ylabel('error \tau')
end

figure,
for i = 1 : 6
subplot(2,3,i)
plot(s_dataset(i,:)'.*180./pi)
title(['Joint ',num2str(i)],'Interpreter','none')
% legend('offline','online')
xlabel('samples')
ylabel('joint pos')
end



figure,
for i = 1 : 6
subplot(2,3,i)
plot(kinDyn_joint_torques(i,:)')
hold on
plot(kinDyn_joint_torques_zero_jointAcc(i,:)')
title(['Joint ',num2str(i)],'Interpreter','none')
legend('acc from KF','acc = 0')
xlabel('samples')
ylabel('\tau')
end


%% Repeat the same but using PID references
s_dataset = datasetStruct{1, 1}.PID.PID;

[sdot_dataset,sddot_dataset] = estimate_joints_vel_acc(s_dataset);

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
for i = 1 : 6
subplot(2,3,i)
plot(kinDyn_joint_torques(i,:)')
hold on
plot(tau_joint(i,:)')
title(['Joint ',num2str(i)],'Interpreter','none')
legend('offline not smoothed','online smoothed')
xlabel('samples')
ylabel('\tau')
end




%% Filter estimated torques

y = lowpass(kinDyn_joint_torques,100,300);

figure,
for i = 1 : 6
subplot(2,3,i)
plot(y(i,:)')
hold on
plot(kinDyn_joint_torques(i,:)')
title(['Joint ',num2str(i)],'Interpreter','none')
legend('offline smoothed','online smoothed')
xlabel('samples')
ylabel('\tau')
end

