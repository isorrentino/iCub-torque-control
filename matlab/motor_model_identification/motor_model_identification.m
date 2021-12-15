%% VALIDATION

joint = 25;

load_dataset;

% reg = [mtr_curr, -mtr_vel];
% k = reg \ joint_trq;

% invKtau = k(1);
% kbemf = k(2);
% 
% Ktau = 1/invKtau;

figure,
scatter(time',joint_trq)
hold on
scatter(time',invKtau_ankle_pitch*mtr_curr - kbemf_ankle_pitch*mtr_vel)
xlabel('time')
ylabel('joint torque')
legend('measured','estimated')
title('right ankle pitch')