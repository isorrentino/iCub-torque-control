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
        mtr_vel_deg_sec = [];
    end
    
    joint_trq = [joint_trq;Joint_state.joint_torques(joint,:)'];
    mtr_curr = [mtr_curr;Motor_state.motor_currents(joint,:)'];
    mtr_vel_deg_sec = [mtr_vel_deg_sec;Motor_state.motor_velocities(joint,:)']; % deg/sec
end

mtr_vel_rad_sec = mtr_vel_deg_sec * pi/180; % rad/sec
