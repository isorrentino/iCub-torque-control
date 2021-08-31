list_of_joints = {'hippitch','hiproll','hipyaw','knee','anklepitch','ankleroll'};

% Select the joint to consider
list = {'hippitch','hiproll','hipyaw',...                   
'knee','anklepitch','ankleroll'};
[joint_from_the_list,tf] = listdlg('ListString',list);

gearbox_values = [100.00  -100.00   100.00   100.00  -100.00   -100.00];

select_dataset = true;

if (select_dataset)
  datasetStruct{joint_from_the_list} = load_data(select_dataset);
end

motor_torque = [];
motor_vel = [];
current = [];
for j = 1 : size(datasetStruct{joint_from_the_list},2)
  motor_torque = [motor_torque; datasetStruct{joint_from_the_list}{j}.Joint_state.joint_torques(joint_from_the_list,:)' / gearbox_values(joint_from_the_list)];
  motor_vel = [motor_vel; datasetStruct{joint_from_the_list}{j}.Motor_state.motor_velocities(joint_from_the_list,:)'];
  current = [current; datasetStruct{joint_from_the_list}{j}.Motor_state.motor_currents(joint_from_the_list,:)'];
end

idx = find(abs(current) < 0.05);

current_low_curr = current(idx);
torque_low_curr = motor_torque(idx);
vel_low_curr = motor_vel(idx);

% Consider viscous + coulomb
regressor = [vel_low_curr, sign(vel_low_curr)];

k = regressor \ torque_low_curr;

fv(joint_from_the_list) = k(1);
% display(['fv = ', num2str(fv)]);

fs(joint_from_the_list) = k(2);
% display(['fs = ', num2str(fs)]);

% Consider just viscous (called also bemf in the code)
regressor = vel_low_curr;

kbemf(joint_from_the_list) = regressor \ torque_low_curr;
