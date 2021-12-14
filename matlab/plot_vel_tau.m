list_of_joints = {'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

for i = 1 : 6
joint_from_the_list = i;

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

idx = find(abs(current) < 0.05);

current_low_curr = current(idx);
torque_low_curr = motor_torque(idx);
vel_low_curr = motor_vel(idx);


figure,
scatter(motor_vel(idx),motor_torque(idx),10)
hold on
scatter(-10000:100:10000, fv(joint_from_the_list).*(-10000:100:10000)+fs(joint_from_the_list).*(sign(-10000:100:10000)))
xlabel('\omega_m deg/sec')
ylabel('\tau_m = fs sgn(\dot{q}) + fv \dot{q}')
title(list_of_joints{joint_from_the_list})

figure,
scatter(motor_vel(idx),motor_torque(idx),10)
hold on
scatter(-10000:100:10000, fv(joint_from_the_list).*(-10000:100:10000)+fs(joint_from_the_list).*(sign(-10000:100:10000)))
xlabel('\omega_m deg/sec')
ylabel('\tau_m = kbemf \dot{q}')
title(list_of_joints{joint_from_the_list})

% figure
% plot(current(idx))
% xlabel('samples')
% ylabel('i_m')
% title(list_of_joints{joint_from_the_list})
end
