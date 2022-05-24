addpath('../filters/kf');

config.robot = 'iCubGenova09';

% config.ft_to_calibrate_from_urdf = {'l_arm_ft_sensor','r_arm_ft_sensor'};
% config.ft_to_calibrate_from_dataset =  {'left_arm_ft_client','right_arm_ft_client'};

config.ft_to_calibrate_from_urdf = {'r_arm_ft_sensor'};
config.ft_to_calibrate_from_dataset =  {'right_arm_ft_client'};

config.experiments = {'robot_logger_device_2022_03_30_15_47_35'};

config.dataset_files_location = './data/';

config.estimate_acceleration = true;

config.select_low_velocity_samples = true;

config.use_velocity_and_acceleration = true;

config.contact_link = {'root_link'};
% config.contact_frames = {'l_foot_front','l_foot_rear','r_foot_front','r_foot_rear'};
config.contact_frames = {'root_link'};

config.compute_expected_fts = true;

config.estimate_external_wrenches = true;

config.use_estimated_fts_after_calibration = true;

config.ft_names_urdf = {'l_arm_ft_sensor';'r_arm_ft_sensor';'l_foot_front_ft_sensor'; ...
                       'l_foot_rear_ft_sensor';'r_foot_front_ft_sensor';'r_foot_rear_ft_sensor';'r_leg_ft_sensor'};

config.ft_names_yarp = {'left_arm_ft_client';'right_arm_ft_client';'left_front_ft_client'; ...
                       'left_rear_ft_client';'right_front_ft_client';'right_rear_ft_client';'right_upper_leg_ft_client'};
                   
config.ft_ext_wrench_frames = {'l_foot_front','l_foot_rear','r_foot_front','r_foot_rear','r_upper_leg','l_upper_arm','r_upper_arm'};

config.scaling_opt = false;

config.use_wbd_data = false;




