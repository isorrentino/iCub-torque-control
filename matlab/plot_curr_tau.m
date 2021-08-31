
joint_from_the_list

current_col = {};
motor_torque_col = {};
motor_vel_col = {};
idx = {};
joint_pos = {};

for j = 1 : size(datasetStruct,2)
  current_col{j} = datasetStruct{j}.Motor_state.motor_currents(joint_from_the_list,:)';
  motor_torque_col{j} = datasetStruct{j}.Joint_state.joint_torques(joint_from_the_list,:)' / gearbox_values(joint_from_the_list);
  motor_vel_col{j} = datasetStruct{j}.Motor_state.motor_velocities(joint_from_the_list,:)';
  
  joint_pos{j} = datasetStruct{j}.Joint_state.joint_positions(joint_from_the_list,1);
  
  idx{j} = find(abs(motor_vel_col{:,j}) < 50);
end


figure
for j = 1 : size(datasetStruct, 2)
  cc = repmat(j,length(motor_vel_col{j}(idx{j})),1);
  scatter(current_col{j}(idx{j}),motor_torque_col{j}(idx{j}),10)
  hold on
end
xlabel('i_m')
ylabel('\tau_m')
title(list_of_joints{joint_from_the_list})
legend(['q = ', num2str(joint_pos{1}(1)*180/pi)],['q = ', num2str(joint_pos{2}(1)*180/pi)],['q = ', num2str(joint_pos{3}(1)*180/pi)])


figure
for j = 1 : size(datasetStruct, 2)
  plot(motor_vel_col{j}(idx{j}))
  hold on
end
xlabel('samples')
ylabel('\omega_m deg/sec')
title(list_of_joints{joint_from_the_list})
