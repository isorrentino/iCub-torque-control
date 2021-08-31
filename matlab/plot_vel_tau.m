for i = 1 : 6
joint_from_the_list = i;

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


figure,
scatter(motor_vel(idx),motor_torque(idx),10)
hold on
scatter(-20000:100:20000, fv(joint_from_the_list).*(-20000:100:20000)+fs(joint_from_the_list).*(sign(-20000:100:20000)))
xlabel('\omega_m deg/sec')
ylabel('\tau_m')
title(list_of_joints{joint_from_the_list})

% figure
% plot(current(idx))
% xlabel('samples')
% ylabel('i_m')
% title(list_of_joints{joint_from_the_list})
end
