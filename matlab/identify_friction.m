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
motor_vel = [];
current = [];
for j = 1 : size(datasetStruct{joint_from_the_list},2)
  motor_torque = [motor_torque; datasetStruct{joint_from_the_list}{j}.Joint_state.joint_torques(idx_joint,:)' / gearbox_values(joint_from_the_list)];
  motor_vel = [motor_vel; datasetStruct{joint_from_the_list}{j}.Motor_state.motor_velocities(idx_joint,:)'];
  current = [current; datasetStruct{joint_from_the_list}{j}.Motor_state.motor_currents(idx_joint,:)'];
end

idx = find(abs(current) < 1);

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


figure,
scatter(vel_low_curr,torque_low_curr)
hold on
scatter(min(vel_low_curr):0.01:max(vel_low_curr),(min(vel_low_curr):0.01:max(vel_low_curr))*kbemf(joint_from_the_list))
xlabel('motor velocity')
ylabel('\tau friction (kbemf dotq)')


figure,
scatter(vel_low_curr,torque_low_curr)
hold on
scatter(min(vel_low_curr):0.01:max(vel_low_curr),(min(vel_low_curr):0.01:max(vel_low_curr))*fv(joint_from_the_list)+sign(min(vel_low_curr):0.01:max(vel_low_curr))*fs(joint_from_the_list))
xlabel('motor velocity')
ylabel('\tau friction (fv dotq + fs sng(dot q))')


