% close all;

new_wbd = false;
use_vel_meas = false;

if new_wbd
    joint = 27;
    
    joint_pos = reshape(robot_logger_device.walking.wbd.kf.position.data,32,[]);
    joint_vel = reshape(robot_logger_device.walking.wbd.kf.velocity.data,32,[]);
    joint_acc = reshape(robot_logger_device.walking.wbd.kf.acceleration.data,32,[]);
    joint_pos_kf = reshape(robot_logger_device.walking.wbd.kf.positionKF.data,32,[]);
    joint_vel_kf = reshape(robot_logger_device.walking.wbd.kf.velocityKF.data,32,[]);
    joint_acc_kf = reshape(robot_logger_device.walking.wbd.kf.accelerationKF.data,32,[]);
    
    x_meas_kf = joint_pos_kf(joint,:)';
    dx_meas_kf = joint_vel_kf(joint,:)';
    ddx_meas_kf = joint_acc_kf(joint,:)';
    ddx_meas = joint_acc(joint,:)';
else
    joint = 21;
    
    joint_pos = reshape(robot_logger_device.joints_state.positions.data,26,[]);
    joint_vel = reshape(robot_logger_device.joints_state.velocities.data,26,[]);
end

fs = 100;

x = joint_pos(joint,:)';
% Numeric derivative
dx_numeric = [0; diff(x)*fs];
x_meas = joint_pos(joint,:);
dx_meas = joint_vel(joint,:)';



% Compute ground truth
% Design low pass filter
fcmin = 5;
fcmax = 8;
order = 10;

% d = designfilt('lowpassfir','FilterOrder',order, ...
%     'PassbandFrequency',fcmin, 'StopbandFrequency', fcmax, ...
%     'SampleRate',fs);


fx = 0.15;
d = designfilt('lowpassiir','FilterOrder',order, ...
    'HalfPowerFrequency',fx,'DesignMethod','butter');

% Filter signal using Zero-phase digital filtering filtfilt
dx_filt = filtfilt(d,dx_meas); 

% Plot fitlered signal
% figure
% plot(dx_numeric)
% hold on
% plot(dx_meas,'LineWidth',3)
% plot(dx_filt,'LineWidth',2)
% legend('Numeric derivative','Measured velocity','Zero-Phase Filtering')
% movegui('northwest')



ddx_numeric = [0; diff(dx_meas)*fs];

% Filter signal using Zero-phase digital filtering filtfilt
ddx_filt = filtfilt(d,ddx_numeric); 

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
x0 = [x(1,:); 0; 0];

if use_vel_meas
    q = [1e-3; 1e-3; 1e1];
    r = [1e-3; 1e-1];
    kf = initKFNullJerk(x0, q, r, 1/fs);
    
    % Filter signal
    for sample = 1 : size(x,1)
      kf.z = [x(sample); dx_meas(sample)];
      kf = kalmanfilter(kf);
      xh(:,sample) = kf.x;
    end
else
    q = [1e-3; 1e-1; 1e3];
    r = 1e-3;
    kf = initKFNullJerk(x0, q, r, 1/fs);
    
    % Filter signal
    for sample = 1 : size(x,1)
      kf.z = x(sample);
      kf = kalmanfilter(kf);
      xh(:,sample) = kf.x;
    end
end
dx_kf(:,1) = xh(2,:);
ddx_kf(:,1) = xh(3,:);

% Plot fitlered signal
% Plot fitlered signal
figure
subplot(2,1,1)
hold on
plot(dx_meas,'LineWidth',2)
plot(dx_filt,'LineWidth',2)
plot(dx_kf,'LineWidth',2)
hold off
title('Velocity')
legend('Measured','Zero-Phase Filtering','KF')
movegui('northwest')

subplot(2,1,2)
hold on
plot(ddx_filt,'LineWidth',2)
plot(ddx_kf,'LineWidth',2)
hold off
title('Acceleration')
legend('Zero-Phase Filtering','KF')
movegui('northwest')


if new_wbd
    figure,
    subplot(2,1,1)
    plot(dx_kf)
    hold on
    plot(dx_meas_kf)
    title('Vel kf offline - online')

    subplot(2,1,2)
    plot(ddx_kf)
    hold on
    plot(ddx_meas_kf)
    title('Acc kf offline - online')
end



