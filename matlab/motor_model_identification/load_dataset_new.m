for ii = 1 : num_datasets
    if ii==1
        [file,path] = uigetfile('~/dev/Datasets/*.mat','Select dataset for identification');
    else
        [file,path] = uigetfile(path,'Select dataset for identification');
    end
    matToLoad = [path, file];
    load(matToLoad);
    
    if ii == 1
        joint_trq = [];
        mtr_curr_A = [];
        mtr_vel_rad_sec = [];
        mtr_pos = [];
        joint_pos = [];
        joint_vel = [];
    end
    
    joint_trq = [joint_trq;robot_logger_device.joints_state.torques.data(joint,:)'];
    mtr_curr_A = [mtr_curr_A;robot_logger_device.motors_state.currents.data(joint,:)']; % Amp
    mtr_vel_rad_sec = [mtr_vel_rad_sec;robot_logger_device.motors_state.velocities.data(joint,:)']; % rad/sec
    mtr_pos = [mtr_pos;180/pi * robot_logger_device.motors_state.positions.data(joint,:)'];
    joint_pos = [joint_pos;180/pi * robot_logger_device.joints_state.positions.data(joint,:)'];
    joint_vel = [joint_vel;180/pi * robot_logger_device.joints_state.velocities.data(joint,:)'];
end

mtr_vel_deg_sec = mtr_vel_rad_sec * 180/pi; % deg/sec
mtr_curr_mA = mtr_curr_A * 1000; % mAmp