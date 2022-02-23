% close all;

joint = 27;

fs = 100;

joint_pos = reshape(robot_logger_device.walking.wbd.kf.position.data,32,[]);
joint_vel = reshape(robot_logger_device.walking.wbd.kf.velocity.data,32,[]);
joint_acc = reshape(robot_logger_device.walking.wbd.kf.acceleration.data,32,[]);
joint_pos_kf = reshape(robot_logger_device.walking.wbd.kf.positionKF.data,32,[]);
joint_vel_kf = reshape(robot_logger_device.walking.wbd.kf.velocityKF.data,32,[]);
joint_acc_kf = reshape(robot_logger_device.walking.wbd.kf.accelerationKF.data,32,[]);

% Number of joints
n = size(joint_pos,1);

% Compute ground truth
% Design low pass filter
fcmin = 5;
fcmax = 8;
order = 10;

% d = designfilt('lowpassfir','FilterOrder',order, ...
%     'PassbandFrequency',fcmin, 'StopbandFrequency', fcmax, ...
%     'SampleRate',fs);


fx = 0.15;

ddx_numeric = [zeros(n,1), (diff(joint_vel')*fs)'];

for i = 1 : n
    d = designfilt('lowpassiir','FilterOrder',order, ...
        'HalfPowerFrequency',fx,'DesignMethod','butter');

    % Filter signal using Zero-phase digital filtering filtfilt
    
    dx_filt(i,:) = filtfilt(d,joint_vel(i,:)'); 

    % Filter signal using Zero-phase digital filtering filtfilt
    ddx_filt(i,:) = filtfilt(d,ddx_numeric(i,:)');
end


% Plot fitlered signal
% figure
% plot(ddx_numeric)
% hold on
% plot(ddx_filt,'LineWidth',3)
% title('Acceleration')
% legend('Numeric derivative','Zero-Phase Filtering')
% movegui('northwest')



%% KF

% Init filter
x0 = [joint_pos(:,1); zeros(2*n,1)];


q = [repmat(1e-3,n,1); repmat(1e-3,n,1); repmat(1e1,n,1)];
r = repmat(1e-3,n,1);

kf = initKFNullJerk(x0, q, r, 1/fs);

% Filter signal
for sample = 1 : size(joint_pos,2)
  kf.z = joint_pos(:,sample);
  kf = kalmanfilter(kf);
  xh(:,sample) = kf.x;
end

dx_kf = xh(n+1:2*n,:);
ddx_kf = xh(2*n+1:end,:);

% Plot fitlered signal
% Plot fitlered signal
figure
subplot(2,1,1)
hold on
plot(joint_vel(joint,:)','LineWidth',2)
plot(dx_filt(joint,:)','LineWidth',2)
plot(dx_kf(joint,:)','LineWidth',2)
hold off
title('Velocity')
legend('Measured','Zero-Phase Filtering','KF')
movegui('northwest')

subplot(2,1,2)
hold on
plot(ddx_filt(joint,:)','LineWidth',2)
plot(ddx_kf(joint,:)','LineWidth',2)
hold off
title('Acceleration')
legend('Zero-Phase Filtering','KF')
movegui('northwest')


figure,
subplot(2,1,1)
plot(dx_kf(joint,:)')
hold on
plot(joint_vel_kf(joint,:)')
legend('offline','online')
title('Vel kf offline - online')

subplot(2,1,2)
plot(ddx_kf(joint,:)')
hold on
plot(joint_acc_kf(joint,:)')
legend('offline','online')
title('Acc kf offline - online')




