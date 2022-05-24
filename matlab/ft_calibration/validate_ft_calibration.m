clc;
clear;

% Preliminary actioins

config_ft_calibration;

experiment_validation = 'robot_logger_device_2022_03_28_11_42_50';

experiment_calibration = 'robot_logger_device_2022_03_28_11_42_50';

% ft_to_calibrate = {'l_arm_ft_sensor','r_arm_ft_sensor','r_foot_front_ft_sensor','r_foot_rear_ft_sensor','l_foot_front_ft_sensor','l_foot_rear_ft_sensor','r_leg_ft_sensor'};
ft_to_calibrate = {'r_leg_ft_sensor'};

experiment_location = './data/parsed/';

mat_calibration_matrix = strcat('./calibration_results/',experiment_validation,'/calibration_results.mat');

load(strcat(experiment_location,experiment_calibration,'_parsed.mat'));

results = load(mat_calibration_matrix);


% Prepare data for estimation

for i = 1 : length(ft_to_calibrate)
    measured_ft = dataset.ft_values.(ft_to_calibrate{i});
    estimated_ft.(ft_to_calibrate{i}) = (results.sol.(ft_to_calibrate{i}).C * measured_ft' - results.sol.(ft_to_calibrate{i}).o)';
end

for j = 1 : length(config.ft_names_urdf)
    if isempty(find(strcmp(config.ft_names_urdf{j},config.ft_to_calibrate_from_urdf),1))
        estimated_ft.(config.ft_names_urdf{j}) = dataset.expected_fts.(config.ft_names_urdf{j});
    end
end

% Estimate external wrenches using expected fts

dataset = compute_expected_contact_wrenches(config, dataset, dataset.expected_fts);

dataset.tested_external_wrenches_expected_ft = dataset.tested_external_wrenches;

% Estimate external wrenches using fts estimated after calibration

dataset = compute_expected_contact_wrenches(config, dataset, estimated_ft);

dataset.tested_external_wrenches_measured_ft = dataset.tested_external_wrenches;

figure,
plot(dataset.tested_external_wrenches_expected_ft.r_foot_front(:,1:3))
hold on
plot(dataset.tested_external_wrenches_measured_ft.r_foot_front(:,1:3))
% legend('case expected ft', 'case estimated ft after calibration')
xlabel('samples')
ylabel('ext wrenches')


% Estimate external wrenches


