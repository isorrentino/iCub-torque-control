clear
close all

num_datasets = 5;

joint = 24;

load_dataset_new;

mtr_acc_deg_sec_2(1:2,1) = 0;
for i = 3:length(mtr_vel_deg_sec)
    mtr_acc_deg_sec_2(i) = 0.99 * mtr_acc_deg_sec_2(i-1) + 0.01 * (mtr_vel_deg_sec(i) - mtr_vel_deg_sec(i-1))*100;
end


net = feedforwardnet(5);
net.numinputs = 2;
x = {mtr_vel_deg_sec'; mtr_curr_mA'};%; mtr_acc_deg_sec_2'; mtr_pos'; joint_pos'; joint_vel';};
y = joint_trq';
net = configure(net,x);
net = train(net,x,y);
y2 = net(x);
figure,scatter(1:length(joint_trq),joint_trq),hold on,scatter(1:length(y2{1}),y2{1})



%% Validation

load_dataset_new;

mtr_acc_deg_sec_2 = [];
mtr_acc_deg_sec_2(1:2,1) = 0;
for i = 3:length(mtr_vel_deg_sec)
    mtr_acc_deg_sec_2(i) = 0.99 * mtr_acc_deg_sec_2(i-1) + 0.01 * (mtr_vel_deg_sec(i) - mtr_vel_deg_sec(i-1))*100;
end

x = {mtr_vel_deg_sec'; mtr_curr_mA'};%; mtr_acc_deg_sec_2'; mtr_pos'; joint_pos'; joint_vel';};
y3 = net(x);

figure,plot(1:length(joint_trq),joint_trq),hold on,scatter(1:length(y3{1}),y3{1})

disp('RMSE')
RMSE = sqrt(mean(joint_trq' - y3{1}).^2);
disp(RMSE)

