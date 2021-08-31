list_of_joints = {'hippitch','hiproll','hipyaw','knee','anklepitch','ankleroll'};

% Select the joint to consider
list = {'hippitch','hiproll','hipyaw',...                   
'knee','anklepitch','ankleroll'};
[joint_from_the_list,tf] = listdlg('ListString',list);

gearbox_values = [100.00  -100.00   100.00   100.00  -100.00   -100.00];

select_dataset = true;
if (select_dataset)
  datasetStruct = load_data(select_dataset);
end

% The identification is motor side
% So we identify ktau such that tau_m = k_tau i_m
current = [];
motor_torque = [];
motor_vel = [];
for j = 1 : size(datasetStruct,2)
  current = [current; datasetStruct{j}.Motor_state.motor_currents(joint_from_the_list,:)'];
  motor_torque = [motor_torque; datasetStruct{j}.Joint_state.joint_torques(joint_from_the_list,:)' / gearbox_values(joint_from_the_list)];
  motor_vel = [motor_vel; datasetStruct{j}.Motor_state.motor_velocities(joint_from_the_list,:)'];
end

idx = find(abs(motor_vel) < 50);

current_low_vel = current(idx);
torque_low_vel = motor_torque(idx);

regressor = current_low_vel;

k = regressor \ torque_low_vel;

k_tau(joint_from_the_list) = k(1);
% i_offset = k(2);
