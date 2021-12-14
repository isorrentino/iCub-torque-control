list_of_joints = {'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

% Select the joint to consider
% list = {'hippitch','hiproll','hipyaw',...                   
% 'knee','anklepitch','ankleroll'};
[joint_from_the_list,tf] = listdlg('ListString',list_of_joints);

gearbox_values = [100.00  -100.00   100.00   100.00  100.00   100.00];

select_dataset = true;

if (select_dataset)
  datasetStruct{joint_from_the_list} = load_data(select_dataset);
end

idx_joint = strcmp(datasetStruct{joint_from_the_list}{1}.Joint_state.joints,list_of_joints{joint_from_the_list});
idx_joint = find(idx_joint);

motor_torque = [];
current = [];
motor_vel = [];
for j = 1 : size(datasetStruct{joint_from_the_list},2)
  motor_torque = [motor_torque; datasetStruct{joint_from_the_list}{j}.Joint_state.joint_torques(idx_joint,:)' / gearbox_values(joint_from_the_list)];
  current = [current; datasetStruct{joint_from_the_list}{j}.Motor_state.motor_currents(idx_joint,:)'];
  motor_vel = [motor_vel; datasetStruct{joint_from_the_list}{j}.Motor_state.motor_velocities(idx_joint,:)'];
end

% regressor = [motor_vel, sign(motor_vel), current];
regressor = [motor_vel, current];

k = regressor \ motor_torque;

fv = k(1);
% fs = k(2);
% k_tau = k(3);
k_tau = k(2);


figure,
scatter(motor_vel,motor_torque-k_tau*current)
hold on
scatter(min(motor_vel):0.01:max(motor_vel),(min(motor_vel):0.01:max(motor_vel))*fv)





