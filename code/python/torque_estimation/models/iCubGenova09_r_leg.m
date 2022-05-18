% Joints limits from robots_configuration/
% page 16

% Positions
q_lb = [-0.7854   -0.1745   -1.3963   -1.2217   -0.7854   -0.4363];
q_ub = [2.0071    2.0071    1.3963    0.0873    0.7854    0.4363];

% Velocities
dq_abs_b = [2.0944   2.0944   2.0944   2.0944   2.0944   2.0944];


% Joints number
n_joints = 6;

% Sampling time
robotSamplingPeriod = 0.01;

% Base link
baseLink = 'r_hip_1';

% Joint names
jointNameslist = {
                            'r_hip_pitch';
                            'r_hip_roll';
                            'r_hip_yaw';
                            'r_knee';
                            'r_ankle_pitch';
                            'r_ankle_roll';
                          };

% Home joint position
homePosition = [34.9991   52.5001         0  -32.4982         0         0];

% Number of degrees of freedom
nrOfDOFs = n_joints;

% Torque constant from datasheet https://github.com/robotology/community/discussions/490
Kt = diag([0.111  0.047  0.047  0.111  0.111  0.025]);

% Coupling matrix from configuration files
% https://github.com/robotology/robots-configuration/blob/master/iCubGenova09/hardware/mechanicals/right_leg-eb10-j0_1-mec.xml
% https://github.com/robotology/robots-configuration/blob/master/iCubGenova09/hardware/mechanicals/right_leg-eb11-j2_3-mec.xml
% https://github.com/robotology/robots-configuration/blob/master/iCubGenova09/hardware/mechanicals/right_leg-eb12-j4_5-mec.xml
K_coup = eye(nrOfDOFs);

% Gear ratio matrix from configuration files
% https://github.com/robotology/robots-configuration/blob/master/iCubGenova09/hardware/mechanicals/right_leg-eb10-j0_1-mec.xml
% https://github.com/robotology/robots-configuration/blob/master/iCubGenova09/hardware/mechanicals/right_leg-eb11-j2_3-mec.xml
% https://github.com/robotology/robots-configuration/blob/master/iCubGenova09/hardware/mechanicals/right_leg-eb12-j4_5-mec.xml
Kr = diag([100  -100  100  100  100  100]);

% Transmission matrix accounting for the gearbox and the coupling
K_gear_coup = K_coup * Kr;

