for ii = 1 : num_datasets
    if load_dataset_bool
        [file,path] = uigetfile('*.mat','Select dataset for identification');
        matToLoad = [path, file];
        load(matToLoad);
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

    joint_trq = [joint_trq;robot_logger_device.joints_state.torques.data(joint,:)'];
    mtr_curr = [mtr_curr;robot_logger_device.motors_state.currents.data(joint,:)'];
    mtr_vel_rad_sec = [mtr_vel_rad_sec;robot_logger_device.motors_state.velocities.data(joint,:)' * pi/180]; % rad/sec
end

mtr_vel_deg_sec = mtr_vel_rad_sec * 180/pi; % deg/sec    
