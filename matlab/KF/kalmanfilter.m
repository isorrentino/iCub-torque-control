function kf = kalmanfilter(kf)

% USAGE:
%
% kf = kalmanfilter(kf)
%
% "kf" is a struct containing various fields used as input
% and output. The state estimate "xh" and its covariance "Px" are
% updated by the function. The other fields describe the mechanics
% of the system and are left unchanged. A calling routine may change
% these other fields as needed if state dynamics are time-dependent;
% otherwise, they should be left alone after initial values are set.
% The exceptions are the observation vector "z" and the input control
% (or forcing function) "u." If there is an input function, then
% "u" should be set to some nonzero value by the calling routine.
%
% SYSTEM DYNAMICS:
%
% The system evolves according to the following difference equations,
% where quantities are further defined below:
%
% x(k) = Ax(k-1) + Bu(k-1) + w  meaning the state vector x evolves during one time
%                               step by premultiplying by the "state transition
%                               matrix" A. There is optionally (if nonzero) an input
%                               vector u which affects the state linearly, and this
%                               linear effect on the state is represented by
%                               premultiplying by the "input matrix" B. There is also
%                               gaussian process noise w.
% z(k-1) = Hx(k-1) + v          meaning the observation vector z is a linear function
%                               of the state vector, and this linear relationship is
%                               represented by premultiplication by "observation
%                               matrix" H. There is also gaussian measurement
%                               noise v.
% where w ~ N(0,Q)              meaning w is gaussian noise with covariance Q
%       v ~ N(0,R)              meaning v is gaussian noise with covariance R
%
% VECTOR VARIABLES:
%
% kf.x = state vector estimate. In the input struct, this is the
%       "a priori" state estimate (prior to the addition of the
%       information from the new observation). In the output struct,
%       this is the "a posteriori" state estimate (after the new
%       measurement information is included).
% kf.z = observation vector
% kf.u = input control vector, optional (defaults to zero).
%
% MATRIX VARIABLES:
%
% kf.A = state transition matrix (defaults to identity).
% kf.Px = covariance of the state vector estimate. In the input struct,
%       this is "a priori," and in the output it is "a posteriori."
%       (required unless autoinitializing as described below).
% kf.B = input matrix, optional (defaults to zero).
% kf.Q = process noise covariance (defaults to zero).
% kf.R = measurement noise covariance (required).
% kf.H = observation matrix (defaults to identity).

% set defaults for absent fields:
if ~isfield(kf,'x'); kf.x = nan*z; end
if ~isfield(kf,'P'); kf.P = nan; end
if ~isfield(kf,'z'); error('Observation vector missing'); end
if ~isfield(kf,'u'); kf.u = 0; end
if ~isfield(kf,'A'); kf.A = eye(length(x)); end
if ~isfield(kf,'B'); kf.B = 0; end
if ~isfield(kf,'Q'); kf.Q = zeros(length(x)); end
if ~isfield(kf,'R'); error('Observation covariance missing'); end
if ~isfield(kf,'H'); kf.H = eye(length(x)); end

% If an initial state estimate is unavailable, it can be obtained
% from the first observation as follows, provided that there are the
% same number of observable variables as state variables. This "auto-
% intitialization" is done automatically if s.x is absent or NaN.
%
% x = inv(H)*z
% P = inv(H)*R*inv(H')
%
% This is mathematically equivalent to setting the initial state estimate
% covariance to infinity.

if isnan(kf.x)
  % initialize state estimate from first observation
  if diff(size(kf.H))
    error('Observation matrix must be square and invertible for state autointialization.');
  end
  kf.x = kf.H \ kf.z;
  kf.Px = kf.H \ (kf.R / kf.H');
else
  % This is the code which implements the discrete Kalman filter:
  
  %---------- Time Update ----------
  % A priori estimate of the current state ( x(t|t-1) = A*x(t-1|t-1) )
  xpred = kf.A * kf.x;
  
  % A priori estimate of the state covariance matrix ( P(t|t-1) = A*P(t-1|t-1)*A' + Q )
  Pxpred = kf.A*kf.Px*kf.A' + kf.Q;
  
  %---------- Measurement Update ----------
  % Kalman filter coefficient ( K(t) = P(t|t-1) * H' * inv(H*P(t|t-1) * H' + R) )
  K = Pxpred * kf.H' / (kf.H*Pxpred*kf.H' + kf.R);
  
  % Estimated observation ( z(t|t-1) = H*x(t|t-1) + R )
  zpred = kf.H * xpred + kf.R;
  
  % Measurement residual error or innovation error ( z(t) - z(t|t-1) )
  resErr = kf.z - zpred;
  
  % A posteriori (updated) estimate of the current state ( x(t|t) = x(t|t-1) + K(t)*(z(t)-z(t|t-1)) )
  kf.x = xpred + K * resErr;
  
  % A posteriori (updated) state covariance matrix ( P(t|t) = (I - K(t)*C) * P(t|t-1) )
  kf.Px = Pxpred - K*kf.H*Pxpred;
end

end
