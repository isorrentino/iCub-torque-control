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



