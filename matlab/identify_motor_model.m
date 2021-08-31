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

motor_torque = [];
current = [];
motor_vel = [];
for j = 1 : size(datasetStruct,2)
  motor_torque = [motor_torque; datasetStruct{j}.Joint_state.joint_torques(joint_from_the_list,:)' / gearbox_values(joint_from_the_list)];
  current = [current; datasetStruct{j}.Motor_state.motor_currents(joint_from_the_list,:)'];
  motor_vel = [motor_vel; datasetStruct{j}.Motor_state.motor_velocities(joint_from_the_list,:)'];
end

% regressor = [motor_vel, sign(motor_vel), current];
regressor = [motor_vel, current];

k = regressor \ motor_torque;

fv = k(1);
% fs = k(2);
% k_tau = k(3);
k_tau = k(2);
