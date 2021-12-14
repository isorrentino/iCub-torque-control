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

% The identification is motor side
% So we identify ktau such that tau_m = k_tau i_m
current = [];
motor_torque = [];
motor_vel = [];
for j = 1 : size(datasetStruct{joint_from_the_list},2)
  current = [current; datasetStruct{joint_from_the_list}{j}.Motor_state.motor_currents(idx_joint,:)'];
  motor_torque = [motor_torque; datasetStruct{joint_from_the_list}{j}.Joint_state.joint_torques(idx_joint,:)' / gearbox_values(joint_from_the_list)];
  motor_vel = [motor_vel; datasetStruct{joint_from_the_list}{j}.Motor_state.motor_velocities(idx_joint,:)'];
end

idx = find(abs(motor_vel) < 10);

current_low_vel = current(idx);
torque_low_vel = motor_torque(idx);

regressor = current_low_vel;

k = regressor \ torque_low_vel;

k_tau(joint_from_the_list) = k(1);
% i_offset = k(2);
