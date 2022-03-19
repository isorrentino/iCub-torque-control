% close all;
clear all;
% clc;

config_ft_calibration;

dataset = parse_robot_logger_device_data_fts_from_wbd(config);

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
    dataset = compute_expected_ft_wrenches_floating_base(config, dataset, estimator);
else
    load(strcat('./data/parsed/',config.experiment,'_parsed.mat'));
end

mkdir(strcat('./calibration_results/',config.experiment));


for i = 1 : length(config.ft_to_calibrate_from_urdf)
    
    %% Calibrate ft left arm
    sol.(config.ft_to_calibrate_from_dataset{i}) = calibrate_ft(dataset.expected_fts.(config.ft_to_calibrate_from_urdf{i}), dataset.ft_values.(config.ft_to_calibrate_from_urdf{i}));
    
    % dataset = validate_calibration([], o, dataset.ft_values.left_arm_ft_client);
    
    measured_ft = dataset.ft_values.(config.ft_to_calibrate_from_urdf{i});
    
    dataset.estimated_ft.(config.ft_to_calibrate_from_dataset{i}) = zeros(size(measured_ft'));
    
    for j = 1 : size(measured_ft,1)
        dataset.estimated_ft.(config.ft_to_calibrate_from_dataset{i})(:,j) = sol.(config.ft_to_calibrate_from_dataset{i}).C * measured_ft(j,:)' - sol.(config.ft_to_calibrate_from_dataset{i}).o;
    end
    
    dataset.estimated_ft.(config.ft_to_calibrate_from_urdf{i}) = dataset.estimated_ft.(config.ft_to_calibrate_from_dataset{i})';
    
    writematrix(sol.(config.ft_to_calibrate_from_dataset{i}).C,strcat('./calibration_results/',config.experiment,'/C_',config.ft_to_calibrate_from_urdf{i},'.txt'));
    writematrix(sol.(config.ft_to_calibrate_from_dataset{i}).o',strcat('./calibration_results/',config.experiment,'/offset_',config.ft_to_calibrate_from_urdf{i},'.txt'));
    save(strcat('./calibration_results/',config.experiment,'/calibration_results.mat'),'sol');
    
    figure,
    plot3(dataset.ft_values.(config.ft_to_calibrate_from_urdf{i})(:,1),...
        dataset.ft_values.(config.ft_to_calibrate_from_urdf{i})(:,2),dataset.ft_values.(config.ft_to_calibrate_from_urdf{i})(:,3))
    hold on
    plot3(dataset.expected_fts.(config.ft_to_calibrate_from_urdf{i})(:,1),dataset.expected_fts.(config.ft_to_calibrate_from_urdf{i})(:,2),dataset.expected_fts.(config.ft_to_calibrate_from_urdf{i})(:,3))
    xlabel('fx')
    ylabel('fy')
    zlabel('fz')
    title(config.ft_to_calibrate_from_urdf{i},'Interpreter','none')


    figure,
    plot3(dataset.estimated_ft.(config.ft_to_calibrate_from_urdf{i})(:,1),dataset.estimated_ft.(config.ft_to_calibrate_from_urdf{i})(:,2),dataset.estimated_ft.(config.ft_to_calibrate_from_urdf{i})(:,3))
    hold on
    plot3(dataset.expected_fts.(config.ft_to_calibrate_from_urdf{i})(:,1),dataset.expected_fts.(config.ft_to_calibrate_from_urdf{i})(:,2),dataset.expected_fts.(config.ft_to_calibrate_from_urdf{i})(:,3))
    xlabel('fx')
    ylabel('fy')
    zlabel('fz')
    title(config.ft_to_calibrate_from_urdf{i},'Interpreter','none')
    
end



for j = 1 : length(config.ft_names_urdf)
    if isempty(find(strcmp(config.ft_names_urdf{j},config.ft_to_calibrate_from_urdf)))
        dataset.estimated_ft.(config.ft_names_urdf{j}) = dataset.expected_fts.(config.ft_names_urdf{i});
    end
end

for j = 1 : length(config.ft_names_urdf)
    dataset.ft_names_urdf.(config.ft_names_urdf{j}) = dataset.ft_values.(config.ft_names_urdf{j});
end

if config.estimate_external_wrenches
    dataset = compute_expected_contact_wrenches(config, dataset, dataset.ft_names_urdf);
else
    load(strcat('./data/parsed/',config.experiment,'_parsed.mat'));
end

figure,
plot(dataset.external_wrenches.l_upper_arm(:,1:3))

