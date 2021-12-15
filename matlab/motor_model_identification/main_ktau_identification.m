% clear;

joint = 25;

load_dataset;

% Identify ktau^-1 (dot theta = 0)
threshold_vel = 0.05;

invKtau_ankle_pitch = mtr_curr(abs(mtr_vel) < threshold_vel) \ joint_trq(abs(mtr_vel) < threshold_vel);
Ktau_ankle_pitch = 1 / invKtau_ankle_pitch;

figure,
scatter(mtr_curr(abs(mtr_vel) < threshold_vel),joint_trq(abs(mtr_vel) < threshold_vel))
hold on
scatter(mtr_curr(abs(mtr_vel) < threshold_vel),invKtau_ankle_pitch*mtr_curr(abs(mtr_vel) < threshold_vel))
xlabel('motor current')
ylabel('joint torque')
legend('measured','estimated')
title('right ankle pitch')


%% ANKLE PITCH
joint = 26;

load_dataset;

% Identify ktau^-1 (dot theta = 0)
threshold_vel = 0.05;

invKtau_ankle_roll = mtr_curr(abs(mtr_vel) < threshold_vel) \ joint_trq(abs(mtr_vel) < threshold_vel);
Ktau_ankle_roll = 1 / invKtau_ankle_roll;

figure,
scatter(mtr_curr(abs(mtr_vel) < threshold_vel),joint_trq(abs(mtr_vel) < threshold_vel))
hold on
scatter(mtr_curr(abs(mtr_vel) < threshold_vel),invKtau_ankle_roll*mtr_curr(abs(mtr_vel) < threshold_vel))
xlabel('motor current')
ylabel('joint torque')
legend('measured','estimated')
title('right ankle roll')

