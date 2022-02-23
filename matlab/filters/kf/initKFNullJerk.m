function kf = initKFNullJerk(x0, q, r, dT)
% The function kf = initKF(x0, stateSize) initializes the variables of the
% kalman filter.

% Model characteristics
% X(k) = A * X(k-1) + v ---> A: State transition matrix, v=N(0,Q) : Process noise
% z(k) = H * X(k)   + w ---> H: Output matrix          , w=N(0,R) : Observation noise

n = length(x0);

% Set covariances
kf.Q = diag(q);  % Process noise
kf.R = diag(r);  % Measurement noise

kf.A = zeros(n,n);

for i = 1 : n/3
    kf.A(i, i) = 1;
    kf.A(i, i+n/3) = dT;
    kf.A(i, i+n/3*2) = 0.5*dT*dT;
    
    kf.A(i+n/3, i+n/3) = 1;
    kf.A(i+n/3, i+n/3*2) = dT;
    
    kf.A(i+n/3*2, i+n/3*2) = 1;
end

if length(r) == 1 
    kf.H = [1 0 0];
elseif length(r) == 2
    kf.H = [eye(2), zeros(2,1)];
end

kf.x = x0;

kf.Px = kf.H \ (kf.R / kf.H');

end
