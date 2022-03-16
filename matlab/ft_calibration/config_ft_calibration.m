config.robot = 'iCubGenova09';

config.ft_name_to_calibrate = 'l_arm_ft_sensor';

config.ft_robot_part_name = 'left_arm';

config.experiment = 'robot_logger_device_2022_03_14_12_14_48_vel_5';

config.dataset_file = strcat('./data/',config.experiment,'.mat');

config.contact_link = 'root_link';

config.compute_expected_fts = false;

config.ft_names_urdf = {'l_arm_ft_sensor';'r_arm_ft_sensor';'l_foot_front_ft_sensor'; ...
                       'l_foot_rear_ft_sensor';'r_foot_front_ft_sensor';'r_foot_rear_ft_sensor';'r_leg_ft_sensor'};



% config.parser_options.forceCalculation = false;
% config.parser_options.saveData = false;
% config.parser_options.testDir = false;
% config.parser_options.filterData = false;
% config.parser_options.raw = true;
% config.parser_options.estimateWrenches = false;
% config.parser_options.useInertial = false;
% config.parser_options.multiSens = false;
% config.parser_options.matFileName = [config.ft_name_to_calibrate,'_parsed_dataset'];


% Variables depending on the urdf

% config.sensor_names = {'l_arm_ft_sensor';'r_arm_ft_sensor';'l_foot_front_ft_sensor'; ...
%                        'l_foot_rear_ft_sensor';'r_foot_front_ft_sensor';'r_foot_rear_ft_sensor';'r_leg_ft_sensor'};
% 
% head = 'head';
% value1 = {'neck_pitch';'neck_roll';'neck_yaw';'eyes_tilt';'eyes_tilt';'eyes_tilt'};
% 
% torso = 'torso'; 
% value2 = {'torso_yaw';'torso_roll';'torso_pitch'};
% 
% left_arm = 'left_arm';
% value3 = {'l_shoulder_pitch';'l_shoulder_roll';'l_shoulder_yaw';'l_shoulder_yaw';'l_shoulder_yaw';'l_shoulder_yaw';'l_shoulder_yaw';'l_hand_finger';...
%     'l_thumb_oppose';'l_thumb_proximal';'l_thumb_distal';'l_index_proximal';'l_index_distal';'l_middle_proximal';'l_middle_distal';' l_pinky'};
% 
% right_arm = 'right_arm';
% value4 = {'r_shoulder_pitch';'r_shoulder_roll';'r_shoulder_yaw';'r_shoulder_yaw';'r_shoulder_yaw';'r_shoulder_yaw';'r_shoulder_yaw';'r_hand_finger';...
%     'r_thumb_oppose';'r_thumb_proximal';'r_thumb_distal';'r_index_proximal';'r_index_distal';'r_middle_proximal';'r_middle_distal';' r_pinky'};
% 
% left_leg = 'left_leg';
% value5 = {'l_hip_pitch';'l_hip_roll';'l_hip_yaw';'l_knee';'l_ankle_pitch';'l_ankle_roll'};
% 
% right_leg = 'right_leg';
% value6 = {'r_hip_pitch';'r_hip_roll';'r_hip_yaw';'r_knee';'r_ankle_pitch';'r_ankle_roll'};
% 
% config.part_names_values = struct( head,       {value1}, ...
%                                    torso,      {value2}, ...
%                                    left_arm,   {value3}, ...
%                                    right_arm,  {value4}, ...
%                                    left_leg,   {value5}, ...
%                                    right_leg,  {value6}     );
                            
                        
                        
                        






