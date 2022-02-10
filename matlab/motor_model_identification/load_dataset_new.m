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
        mtr_curr = [];
        mtr_vel_rad_sec = [];
    end
    
    joint_trq = [joint_trq;robot_logger_device.joints_state.torques.data(joint,:)'];
    mtr_curr = [mtr_curr;robot_logger_device.motors_state.currents.data(joint,:)'];
    mtr_vel_rad_sec = [mtr_vel_rad_sec;robot_logger_device.motors_state.velocities.data(joint,:)']; % rad/sec
end

mtr_vel_deg_sec = mtr_vel_rad_sec * 180/pi; % deg/sec