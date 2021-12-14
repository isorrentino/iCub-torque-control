
joint_from_the_list
idx_joint

current_col = {};
motor_torque_col = {};
motor_vel_col = {};
idx = {};
joint_pos = {};

for j = 1 : size(datasetStruct{joint_from_the_list},2)
  current_col{j} = datasetStruct{joint_from_the_list}{j}.Motor_state.motor_currents(idx_joint,:)';
  motor_torque_col{j} = datasetStruct{joint_from_the_list}{j}.Joint_state.joint_torques(idx_joint,:)' / gearbox_values(joint_from_the_list);
  motor_vel_col{j} = datasetStruct{joint_from_the_list}{j}.Motor_state.motor_velocities(idx_joint,:)';
  
  joint_pos{j} = datasetStruct{joint_from_the_list}{j}.Joint_state.joint_positions(joint_from_the_list,1);
  
  idx{j} = find(abs(motor_vel_col{:,j}) < 10);
end


if size(datasetStruct{joint_from_the_list},2) > 1
    figure
    for j = 1 : size(datasetStruct{joint_from_the_list},2)
      cc = repmat(j,length(motor_vel_col{j}(idx{j})),1);
      scatter(current_col{j}(idx{j}),motor_torque_col{j}(idx{j}),10)
      hold on
    end
    scatter(-8:0.1:8,(-8:0.1:8)*k_tau(joint_from_the_list))
    xlabel('i_m')
    ylabel('\tau_m')
    title(list_of_joints{joint_from_the_list},'Interpreter','none')
%     legend(['q = ', num2str(joint_pos{1}(1)*180/pi)],['q = ', num2str(joint_pos{2}(1)*180/pi)],['q = ', num2str(joint_pos{3}(1)*180/pi)])
else
    figure
    for j = 1 : size(datasetStruct{joint_from_the_list},2)
      cc = repmat(j,length(motor_vel_col{j}(idx{j})),1);
      scatter(current_col{j}(idx{j}),motor_torque_col{j}(idx{j}),10)
      hold on
    end
    scatter(min(current_col{j,1}):0.1:max(current_col{j,1}),(min(current_col{j,1}):0.1:max(current_col{j,1}))*k_tau(joint_from_the_list))
    xlabel('i_m')
    ylabel('\tau_m')
    title(list_of_joints{joint_from_the_list},'Interpreter','none')
end

% figure
% for j = 1 : size(datasetStruct{joint_from_the_list},2)
%   plot(motor_vel_col{j}(idx{j}))
%   hold on
% end
% xlabel('samples')
% ylabel('\omega_m deg/sec')
% title(list_of_joints{joint_from_the_list})
