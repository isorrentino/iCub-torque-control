%% Torque tracking

close all;

time = time - time(1);

figure;
plot(time,r_ankle_pitch_destrq,'DisplayName','r_ankle_pitch_destrq');
hold on;
plot(time,r_ankle_pitch_trq,'DisplayName','r_ankle_pitch_trq');
hold off;
xlabel('time (sec)')
ylabel('\tau')


% Joint position tracking
figure;
plot(time,r_ankle_pitch_despos,'DisplayName','r_ankle_pitch_despos');
hold on;
plot(time,r_ankle_pitch_pos,'DisplayName','r_ankle_pitch_pos');
hold off;
xlabel('time (sec)')
ylabel('q')


% Joint velocity tracking
figure;
plot(time,r_ankle_pitch_desvel,'DisplayName','r_ankle_pitch_desvel');
hold on;
plot(time,r_ankle_pitch_vel,'DisplayName','r_ankle_pitch_vel');
hold off;
xlabel('time (sec)')
ylabel('dq')








