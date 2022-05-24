opti = casadi.Opti();
x = opti.variable();
y = opti.variable();
opti.minimize((1-x)^2+(y-x^2)^2);
opti.solver('ipopt');
sol = opti.solve();

% close all;
clear all;
% clc;

config_ft_calibration;

dataset = parse_robot_logger_device_data_fts(config);

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
    load(strcat('./data/parsed/',config.experiments{1},'_parsed.mat'));
end

mkdir(strcat('./calibration_results/',config.experiments{1}));


for i = 1 : length(config.ft_to_calibrate_from_urdf)
    
    %% Calibrate ft left arm
    sol.(config.ft_to_calibrate_from_urdf{i}) = calibrate_ft(config,dataset.expected_fts.(config.ft_to_calibrate_from_urdf{i})(1:end-1000,:), dataset.ft_values.(config.ft_to_calibrate_from_urdf{i})(1:end-1000,:));
    
    % dataset = validate_calibration([], o, dataset.ft_values.left_arm_ft_client);
    
    measured_ft = dataset.ft_values.(config.ft_to_calibrate_from_urdf{i});
    
    dataset.estimated_ft.(config.ft_to_calibrate_from_dataset{i}) = zeros(size(measured_ft'));
    
    for j = 1 : size(measured_ft,1)
        dataset.estimated_ft.(config.ft_to_calibrate_from_dataset{i})(:,j) = sol.(config.ft_to_calibrate_from_urdf{i}).C * measured_ft(j,:)' - sol.(config.ft_to_calibrate_from_urdf{i}).o;
    end
    
    dataset.estimated_ft.(config.ft_to_calibrate_from_urdf{i}) = dataset.estimated_ft.(config.ft_to_calibrate_from_dataset{i})';
    
    writematrix(sol.(config.ft_to_calibrate_from_urdf{i}).C,strcat('./calibration_results/',config.experiments{1},'/C_',config.ft_to_calibrate_from_urdf{i},'.txt'));
    writematrix(sol.(config.ft_to_calibrate_from_urdf{i}).o',strcat('./calibration_results/',config.experiments{1},'/offset_',config.ft_to_calibrate_from_urdf{i},'.txt'));
    save(strcat('./calibration_results/',config.experiments{1},'/calibration_results.mat'),'sol');
    
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

ft_x = dataset.estimated_ft.r_leg_ft_sensor(:,1);
ft_y = dataset.estimated_ft.r_leg_ft_sensor(:,2);
ft_z = dataset.estimated_ft.r_leg_ft_sensor(:,3);
nn = sqrt(ft_x.^2 + ft_y.^2 + ft_z.^2);
figure,plot(nn)

save(strcat('./data/workspace_calibration/',config.experiments{1}));

