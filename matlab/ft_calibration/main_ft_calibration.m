close all;
clear all;
clc;

config_ft_calibration;

dataset = parse_robot_logger_device_data(config);

% Load the estimator and model information
estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator();

dofs = size(dataset.joint_names,1);

consideredJoints = iDynTree.StringVector();

for i = 1 : dofs
    consideredJoints.push_back(dataset.joint_names{i});
end

for i = 1 : size(config.ft_names_urdf,1)
    consideredJoints.push_back(config.ft_names_urdf{i});
end

estimatorLoader = iDynTree.ModelLoader();
estimatorLoader.loadReducedModelFromFile(strcat('./robots/',config.robot,'.urdf'),consideredJoints);
estimator.setModelAndSensors(estimatorLoader.model(),estimatorLoader.sensors);

if config.compute_expected_fts
    dataset = compute_expected_ft_wrenches(config, dataset, estimator);
else
    load(strcat('./data/parsed/',config.experiment,'_parsed.mat'));
end


for i = 1 : length(config.ft_to_calibrate_urdf)
    
    %% Calibrate ft left arm
    sol.(config.ft_to_calibrata_dataset{i}) = calibrate_ft(dataset.expected_fts.(config.ft_to_calibrate_urdf{i}), dataset.ft_values.(config.ft_to_calibrata_dataset{i}));
    
    % dataset = validate_calibration([], o, dataset.ft_values.left_arm_ft_client);
    
    measured_ft = dataset.ft_values.(config.ft_to_calibrata_dataset{i});
    
    estimated_ft.(config.ft_to_calibrata_dataset{i}) = zeros(size(measured_ft'));
    
    for j = 1 : size(measured_ft,1)
        estimated_ft.(config.ft_to_calibrata_dataset{i})(:,j) = sol.(config.ft_to_calibrata_dataset{i}).C * measured_ft(j,:)' - sol.(config.ft_to_calibrata_dataset{i}).o;
    end
    estimated_ft.(config.ft_to_calibrata_dataset{i}) = estimated_ft.(config.ft_to_calibrata_dataset{i})';
    
    figure,
    plot3(dataset.ft_values.(config.ft_to_calibrata_dataset{i})(:,1),...
        dataset.ft_values.(config.ft_to_calibrata_dataset{i})(:,2),dataset.ft_values.(config.ft_to_calibrata_dataset{i})(:,3))
    hold on
    plot3(dataset.expected_fts.(config.ft_to_calibrate_urdf{i})(:,1),dataset.expected_fts.(config.ft_to_calibrate_urdf{i})(:,2),dataset.expected_fts.(config.ft_to_calibrate_urdf{i})(:,3))
    xlabel('fx')
    ylabel('fy')
    zlabel('fz')
    title('left arm')
    
        
    figure,
    plot3(estimated_ft.(config.ft_to_calibrata_dataset{i})(:,1),estimated_ft.(config.ft_to_calibrata_dataset{i})(:,2),estimated_ft.(config.ft_to_calibrata_dataset{i})(:,3))
    hold on
    plot3(dataset.expected_fts.(config.ft_to_calibrate_urdf{i})(:,1),dataset.expected_fts.(config.ft_to_calibrate_urdf{i})(:,2),dataset.expected_fts.(config.ft_to_calibrate_urdf{i})(:,3))
    xlabel('fx')
    ylabel('fy')
    zlabel('fz')
    title('left arm')
    
    
end





