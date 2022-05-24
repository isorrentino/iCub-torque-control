f0 = 0;
f1 = 0.7;

dT = 0.01;

T = 40;

tinc = 0:dT:T-dT;
tdec = T:-dT:0;

t_tot = 0:dT:2*T;

q0 = 10 * pi / 180;

A = 25 * pi / 180;

% Position

q_inc = q0 + A * sin(2*pi * ((f1-f0)*tinc.^2./(2*T) + f0*tinc));

q_dec = q0 + A * sin(2*pi * ((f0-f1)*tdec.^2./(2*T) + f0*tdec));

traj = [q_inc, q_dec]';

figure,plot(t_tot,[q_inc,q_dec])


% Velocity

dq_inc = A * 2*pi * ((f1-f0)*tinc./T + f0) .* cos(2*pi * ((f1-f0)*tinc.^2./(2*T) + f0*tinc));

dq_dec = - A * 2*pi * ((f0-f1)*tdec./T + f0) .* cos(2*pi * ((f0-f1)*tdec.^2./(2*T) + f0*tdec));

figure,plot(t_tot,[dq_inc,dq_dec]),hold on,plot(t_tot,[0,diff([q_inc,q_dec])/dT])


% Acceleration

ddq_inc = - A * (2*pi * ((f1-f0)*tinc./T + f0)).^2 * (2*pi * (f1-f0) / T) .* sin(2*pi * ((f1-f0)*tinc.^2./(2*T) + f0*tinc));

ddq_dec = A * (2*pi * ((f0-f1)*tdec./T + f0)).^2 * (2*pi * (f0-f1) / T) .* sin(2*pi * ((f0-f1)*tdec.^2./(2*T) + f0*tdec));

figure,plot(t_tot,[ddq_inc,ddq_dec])



% Save trajectory on file
save('chirp.mat','traj');





