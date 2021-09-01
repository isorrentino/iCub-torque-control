function kf = initKF(x0, stateSize)
% The function kf = initKF(x0, stateSize) initializes the variables of the
% kalman filter.

% Model characteristics
% X(k) = A * X(k-1) + v ---> A: State transition matrix, v=N(0,Q) : Process noise
% z(k) = H * X(k)   + w ---> H: Output matrix          , w=N(0,R) : Observation noise

% Init filter
dT = 1/100; % Sampling time

% Set covariances
kf.Q = diag([1e-5; 0.005; 0.1]);  % Process noise
kf.R = 1e-7; %1e-5;  % Measurement noise

kf.A = eye(stateSize);
for ii = 1 : stateSize-1
  for jj = ii+1 : stateSize
    kf.A(ii,jj) = dT^(jj-ii)/factorial(jj-ii);
  end
end

kf.H = [1  zeros(1,stateSize-1)];

kf.x = x0;

kf.Px = kf.H \ (kf.R / kf.H');

end
