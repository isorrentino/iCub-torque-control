function kf = initKFNullJerk(x0, q, r, dT)
% The function kf = initKF(x0, stateSize) initializes the variables of the
% kalman filter.

% Model characteristics
% X(k) = A * X(k-1) + v ---> A: State transition matrix, v=N(0,Q) : Process noise
% z(k) = H * X(k)   + w ---> H: Output matrix          , w=N(0,R) : Observation noise

% The state vector is [pos(nx1), vel(nx1), acc(nx1)]
% and n is the number of joints
n = length(x0)/3;

% Set covariances
kf.Q = diag(q);  % Process noise
kf.R = diag(r);  % Measurement noise

kf.A = zeros(n*3,n*3);

for i = 1 : n
    kf.A(i, i) = 1;
    kf.A(i, i+n) = dT;
    kf.A(i, i+n*2) = 0.5*dT*dT;
    
    kf.A(i+n, i+n) = 1;
    kf.A(i+n, i+n*2) = dT;
    
    kf.A(i+n*2, i+n*2) = 1;
end

kf.H = [eye(n), zeros(n,2*n)];

kf.x = x0;

kf.Px = kf.H \ (kf.R / kf.H');

end
