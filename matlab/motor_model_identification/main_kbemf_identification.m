% clear;

joint = 25;

load_dataset;

% Identify kbemf (im = 0)
threshold_curr = 0.05;

mtr_vel = mtr_vel * 180/pi;

kbemf_ankle_pitch = -mtr_vel(abs(mtr_curr) < threshold_curr) \ joint_trq(abs(mtr_curr) < threshold_curr);

figure,
scatter(mtr_vel(abs(mtr_curr) < threshold_curr),joint_trq(abs(mtr_curr) < threshold_curr))
hold on
scatter(mtr_vel(abs(mtr_curr) < threshold_curr),-kbemf_ankle_pitch*mtr_vel(abs(mtr_curr) < threshold_curr))
xlabel('motor velocity')
ylabel('joint torque')
legend('measured','estimated')
title('right ankle pitch')

idx_neg = find(abs(mtr_curr) < threshold_curr & mtr_vel<=0);
idx_pos = find(abs(mtr_curr) < threshold_curr & mtr_vel>=0);

reg1 = [sign(mtr_vel(idx_neg)), mtr_vel(idx_neg)];
k1 = -reg1 \ joint_trq(idx_neg);

reg2 = [sign(mtr_vel(idx_pos)), mtr_vel(idx_pos)];
k2 = -reg2 \ joint_trq(idx_pos);

figure,
scatter(mtr_vel(abs(mtr_curr) < threshold_curr),joint_trq(abs(mtr_curr) < threshold_curr))
hold on
scatter(mtr_vel(idx_neg),-k1(1)*sign(mtr_vel(idx_neg)) - k1(2)*mtr_vel(idx_neg))
scatter(mtr_vel(idx_pos),-k2(1)*sign(mtr_vel(idx_pos)) - k2(2)*mtr_vel(idx_pos))
xlabel('motor velocity')
ylabel('joint torque')
legend('measured','estimated')
title('right ankle pitch')


%% ANKLE ROLL
joint = 26;

load_dataset;

mtr_vel = mtr_vel * 180/pi;

% Identify kbemf (im = 0)
threshold_curr = 0.05;

kbemf_ankle_roll = -mtr_vel(abs(mtr_curr) < threshold_curr) \ joint_trq(abs(mtr_curr) < threshold_curr);

figure,
scatter(mtr_vel(abs(mtr_curr) < threshold_curr),joint_trq(abs(mtr_curr) < threshold_curr))
hold on
scatter(mtr_vel(abs(mtr_curr) < threshold_curr),-kbemf_ankle_roll*mtr_vel(abs(mtr_curr) < threshold_curr))
xlabel('motor velocity')
ylabel('joint torque')
legend('measured','estimated')
title('right ankle roll')

idx_neg = find(abs(mtr_curr) < threshold_curr & mtr_vel<=0);
idx_pos = find(abs(mtr_curr) < threshold_curr & mtr_vel>=0);

reg1 = [sign(mtr_vel(idx_neg)), mtr_vel(idx_neg)];
k1 = -reg1 \ joint_trq(idx_neg);

reg2 = [sign(mtr_vel(idx_pos)), mtr_vel(idx_pos)];
k2 = -reg2 \ joint_trq(idx_pos);

figure,
scatter(mtr_vel(abs(mtr_curr) < threshold_curr),joint_trq(abs(mtr_curr) < threshold_curr))
hold on
scatter(mtr_vel(idx_neg),-k1(1)*sign(mtr_vel(idx_neg)) - k1(2)*mtr_vel(idx_neg))
scatter(mtr_vel(idx_pos),-k2(1)*sign(mtr_vel(idx_pos)) - k2(2)*mtr_vel(idx_pos))
xlabel('motor velocity')
ylabel('joint torque')
legend('measured','estimated')
title('right ankle roll')