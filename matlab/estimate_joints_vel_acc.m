function [vel,acc] = estimate_joints_vel_acc(s_dataset)

% Expected input size jointsXsamples

  pos = s_dataset';

  % Estimate acceleration
  stateSize = 3;
  sdot_dataset = zeros(size(pos));
  sddot_dataset = zeros(size(pos));
  
  for j = 1 : 6
    x0 = [pos(1,j); zeros(stateSize-1,1)];
    kf = initKF(x0, stateSize);
    % Run filter for entire traj
    xh = zeros(stateSize, size(pos,1));
    for sample = 1 : size(pos,1)
      kf.z = pos(sample,j);
      kf = kalmanfilter(kf);
      xh(:,sample) = kf.x;
    end
    sdot_dataset(:,j) = xh(2,:)';
    sddot_dataset(:,j) = xh(3,:)';
  end

  vel = sdot_dataset';
  acc = sddot_dataset';
  
end

