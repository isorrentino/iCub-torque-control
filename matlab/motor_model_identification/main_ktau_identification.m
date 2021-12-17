%% RIGHT ANKLE PITCH
% clear;

joint = 25;

load_dataset;

% Identify ktau^-1 (dot theta = 0)
threshold_vel = 0.05;

invKtau_ankle_pitch = mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel) \ joint_trq(abs(mtr_vel_rad_sec) < threshold_vel);
Ktau_ankle_pitch = 1000 / invKtau_ankle_pitch;

figure,
scatter(mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel),joint_trq(abs(mtr_vel_rad_sec) < threshold_vel))
hold on
scatter(mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel),invKtau_ankle_pitch*mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel))
xlabel('motor current')
ylabel('joint torque')
legend('measured','estimated')
title('right ankle pitch')


%% RIGHT ANKLE ROLL
joint = 26;

load_dataset;

% Identify ktau^-1 (dot theta = 0)
threshold_vel = 0.05;

invKtau_ankle_roll = mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel) \ joint_trq(abs(mtr_vel_rad_sec) < threshold_vel);
Ktau_ankle_roll = 1000 / invKtau_ankle_roll;

figure,
scatter(mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel),joint_trq(abs(mtr_vel_rad_sec) < threshold_vel))
hold on
scatter(mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel),invKtau_ankle_roll*mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel))
xlabel('motor current')
ylabel('joint torque')
legend('measured','estimated')
title('right ankle roll')


%% LEFT ANKLE PITCH
joint = 19;

load_dataset;

% Identify ktau^-1 (dot theta = 0)
threshold_vel = 0.05;

invKtau_ankle_roll = mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel) \ joint_trq(abs(mtr_vel_rad_sec) < threshold_vel);
Ktau_ankle_roll = 1000 / invKtau_ankle_roll;

figure,
scatter(mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel),joint_trq(abs(mtr_vel_rad_sec) < threshold_vel))
hold on
scatter(mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel),invKtau_ankle_roll*mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel))
xlabel('motor current')
ylabel('joint torque')
legend('measured','estimated')
title('left ankle pitch')


%% LEFT ANKLE ROLL
joint = 20;

load_dataset;

% Identify ktau^-1 (dot theta = 0)
threshold_vel = 0.05;

invKtau_ankle_roll = mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel) \ joint_trq(abs(mtr_vel_rad_sec) < threshold_vel);
Ktau_ankle_roll = 1000 / invKtau_ankle_roll;

figure,
scatter(mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel),joint_trq(abs(mtr_vel_rad_sec) < threshold_vel))
hold on
scatter(mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel),invKtau_ankle_roll*mtr_curr(abs(mtr_vel_rad_sec) < threshold_vel))
xlabel('motor current')
ylabel('joint torque')
legend('measured','estimated')
title('left ankle roll')

