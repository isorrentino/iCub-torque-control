load_dataset_bool = true;
if load_dataset_bool
    cd '/home/isorrentino/dev/Datasets/YarpRobotLogger';
    [file,path] = uigetfile('*.mat','Select dataset for identification');
    matToLoad = [path, file];
    load(matToLoad);
    cd '/home/isorrentino/dev/iCub-torque-control/matlab/motor_model_identification';
end

if ~exist('joint_trq','var')
    joint_trq = [];
end
if ~exist('mtr_curr','var')
    mtr_curr = [];
end
if ~exist('mtr_vel_rad_sec','var')
    mtr_vel_rad_sec = [];
end

joint_trq = [joint_trq;Joint_state.joint_torques(joint,:)'];
mtr_curr = [mtr_curr;Motor_state.motor_currents(joint,:)'];
mtr_vel_rad_sec = [mtr_vel_rad_sec;Motor_state.motor_velocities(joint,:)' * pi/180]; % rad/sec
mtr_vel_deg_sec = mtr_vel_rad_sec * 180/pi; % deg/sec