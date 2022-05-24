function [dataset] = parse_robot_logger_device_data_fts_from_wbd(config)

dataset.timestamp = [];
dataset.q = [];
dataset.dq = [];
dataset.ddq = [];

for i = 1 : length(config.experiments)
    
    % Load dataset
    dataset_file = config.experiments{i};
    load(strcat(config.dataset_files_location,dataset_file,'.mat'));
    
    % Load joint names, ft names, cartesian wrenche names
    dataset.joint_names = robot_logger_device.description_list;
    dataset.ft_names = fieldnames(robot_logger_device.FTs);
    dataset.cartesian_wrench_names = fieldnames(robot_logger_device.cartesian_wrenches);
    
    % Load timestamp
    timestamp = robot_logger_device.joints_state.positions.timestamps;
    
    % Load joint positioon, velocity, acceleration
    q = reshape(robot_logger_device.joints_state.positions.data,size(robot_logger_device.joints_state.positions.data,1),[])';
    dq = reshape(robot_logger_device.joints_state.velocities.data,size(robot_logger_device.joints_state.velocities.data,1),[])';
    
    % Estimate joint acceleration through kalman filter
    if config.estimate_acceleration
        fs = 100;
        
        n_joints = length(dataset.joint_names);
        
        joint_pos = reshape(robot_logger_device.joints_state.positions.data,n_joints,[]);
        x0 = [joint_pos(:,1); zeros(2*n_joints,1)];
        
        
        cov_q = [repmat(1e-2,n_joints,1); repmat(1e-1,n_joints,1); repmat(1e1,n_joints,1)];
        cov_r = repmat(1e-4,n_joints,1);
        
        kf = initKFNullJerk(x0, cov_q, cov_r, 1/fs);
        
        % Filter signal
        for sample = 1 : size(joint_pos,2)
            kf.z = joint_pos(:,sample);
            kf = kalmanfilter(kf);
            xh(:,sample) = kf.x;
        end
        
        dx_kf = xh(n_joints+1:2*n_joints,:);
        ddx_kf = xh(2*n_joints+1:end,:);
        
        dq = dx_kf';
        ddq = ddx_kf';
    else
        ddq = zeros(size(dataset.dq));
    end
    
    % Load ft values
    if config.use_wbd_data
        for j = 1 : size(config.ft_names_urdf,1)
            ft_values.(config.ft_names_urdf{j}) = reshape(robot_logger_device.wbd.fts.(config.ft_names_urdf{j}).filtered.data,6,[])';
        end
    else
        for j = 1 : size(config.ft_names_yarp,1)
            ft_values.(config.ft_names_urdf{j}) = reshape(robot_logger_device.FTs.(config.ft_names_yarp{j}).data,6,[])';
        end
    end
    
    samples_to_discard = 200;
    
    % Take correct samples
    q = q(samples_to_discard:end,:);
    dq = dq(samples_to_discard:end,:);
    ddq = ddq(samples_to_discard:end,:);
    
    % Load cartesian wrenches
    for j = 1 : size(dataset.cartesian_wrench_names,1)
        cartesian_wrench_values.(dataset.cartesian_wrench_names{j}) = reshape(robot_logger_device.cartesian_wrenches.(dataset.cartesian_wrench_names{j}).data,6,[])';
        cartesian_wrench_values.(dataset.cartesian_wrench_names{j}) = cartesian_wrench_values.(dataset.cartesian_wrench_names{j})(samples_to_discard:end,:);
    end
    
    dataset.timestamp = [dataset.timestamp; timestamp(samples_to_discard:end)'];
    dataset.q = [dataset.q; q];
    dataset.dq = [dataset.dq; dq];
    dataset.ddq = [dataset.ddq; ddq];
    if i == 1
        for j = 1 : size(dataset.cartesian_wrench_names,1)
            dataset.cartesian_wrench_values.(dataset.cartesian_wrench_names{j}) = [];
        end
        for j = 1 : size(config.ft_names_urdf,1)
            dataset.ft_values.(config.ft_names_urdf{j}) = [];
        end
    end
    for j = 1 : size(dataset.cartesian_wrench_names,1)
        dataset.cartesian_wrench_values.(dataset.cartesian_wrench_names{j}) = [dataset.cartesian_wrench_values.(dataset.cartesian_wrench_names{j}); cartesian_wrench_values.(dataset.cartesian_wrench_names{j})];
    end
    for j = 1 : size(config.ft_names_urdf,1)
        dataset.ft_values.(config.ft_names_urdf{j}) = [dataset.ft_values.(config.ft_names_urdf{j}); ft_values.(config.ft_names_urdf{j})];
    end
    
    if config.select_low_velocity_samples
        l = 1;
        for k = 1 : size(dataset.dq,1)
            if sum(abs(dq(k,:))) < 1e-2
                idx(l) = k;
                l = l + 1;
            end
        end
        
        dataset.q = dataset.q(idx,:);
        dataset.dq = dataset.dq(idx,:);
        dataset.ddq = dataset.ddq(idx,:);
        for j = 1 : size(dataset.cartesian_wrench_names,1)
            dataset.cartesian_wrench_values.(dataset.cartesian_wrench_names{j}) = dataset.cartesian_wrench_values.(dataset.cartesian_wrench_names{j})(idx,:);
        end
        for j = 1 : size(config.ft_names_urdf,1)
            dataset.ft_values.(config.ft_names_urdf{j}) = dataset.ft_values.(config.ft_names_urdf{j})(idx,:);
        end
        dataset.timestamp = dataset.timestamp(idx);
    end
    
end

