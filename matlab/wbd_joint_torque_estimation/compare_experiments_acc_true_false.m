[file,path] = uigetfile('*.mat','Select dataset for identification');
matToLoad = [path, file];
dataset_false = load(matToLoad);

[file,path] = uigetfile('*.mat','Select dataset for identification');
matToLoad = [path, file];
dataset_true = load(matToLoad);


pos_acc_false = dataset_false.Joint_state.joint_positions(end-5:end,:)';
pos_acc_true = dataset_true.Joint_state.joint_positions(end-5:end,:)';

time_false = 0.01*(0:length(pos_acc_false)-1);
time_true = 0.01*(0:length(pos_acc_true)-1);

figure,
for i = 1 : 6
    subplot(3,2,i)
    plot(time_false,pos_acc_false(:,i))
    hold on
    plot(time_true,pos_acc_true(:,i))
    xlabel('time (sec)')
    ylabel('joint pos (rad)')
    legend('Use acc false','Use acc true')
    title(['Joint ',num2str(i)])
end


trq_acc_false = dataset_false.Joint_state.joint_torques(end-5:end,:)';
trq_acc_true = dataset_true.Joint_state.joint_torques(end-5:end,:)';

figure,
for i = 1 : 6
    subplot(3,2,i)
    plot(time_false,trq_acc_false(:,i))
    xlabel('time (sec)')
    ylabel('joint torque')
    legend(['Joint ',num2str(i)])
    title('Use acc false')
end
figure,
for i = 1 : 6
    subplot(3,2,i)
    plot(time_true,trq_acc_true(:,i))
    xlabel('time (sec)')
    ylabel('joint torque')
    legend(['Joint ',num2str(i)])
    title('Use acc true')
end


vel_acc_false = dataset_false.Joint_state.joint_velocities(end-5:end,:)';
vel_acc_true = dataset_true.Joint_state.joint_velocities(end-5:end,:)';

figure,
for i = 1 : 6
    subplot(3,2,i)
    plot(time_false,vel_acc_false(:,i))
    hold on
    plot(time_true,vel_acc_true(:,i))
    xlabel('time (sec)')
    ylabel('joint vel (rad/sec)')
    legend('Use acc false','Use acc true')
    title(['Joint ',num2str(i)])
end
