function [dataset] = parse_robot_logger_device_data(config)

load(config.dataset_file);


dataset.timestamp = robot_logger_device.joints_state.positions.timestamps;

dataset.joint_names = robot_logger_device.description_list;

dataset.q = reshape(robot_logger_device.joints_state.positions.data,size(robot_logger_device.joints_state.positions.data,1),[])';
dataset.dq = reshape(robot_logger_device.joints_state.velocities.data,size(robot_logger_device.joints_state.velocities.data,1),[])';
dataset.ddq = reshape(robot_logger_device.joints_state.accelerations.data,size(robot_logger_device.joints_state.accelerations.data,1),[])';

dataset.im = reshape(robot_logger_device.motors_state.currents.data,size(robot_logger_device.motors_state.currents.data,1),[])';

dataset.ft_names = fieldnames(robot_logger_device.FTs);

for i = 1 : size(dataset.ft_names,1)
    
    dataset.ft_values.(dataset.ft_names{i}) = reshape(robot_logger_device.FTs.(dataset.ft_names{i}).data,6,[])';
    
end


dataset.cartesian_wrench_names = fieldnames(robot_logger_device.cartesian_wrenches);

for i = 1 : size(dataset.cartesian_wrench_names,1)
    
    dataset.cartesian_wrench_values.(dataset.cartesian_wrench_names{i}) = reshape(robot_logger_device.cartesian_wrenches.(dataset.cartesian_wrench_names{i}).data,6,[])';
    
end


if config.estimate_acceleration
    fs = 100;
    
    n_joints = length(dataset.joint_names);
    
    joint_pos = reshape(robot_logger_device.joints_state.positions.data,n_joints,[]);
    x0 = [joint_pos(:,1); zeros(2*n_joints,1)];
    
    
    q = [repmat(1e-2,n_joints,1); repmat(1e-1,n_joints,1); repmat(1e1,n_joints,1)];
    r = repmat(1e-4,n_joints,1);
    
    kf = initKFNullJerk(x0, q, r, 1/fs);
    
    % Filter signal
    for sample = 1 : size(joint_pos,2)
        kf.z = joint_pos(:,sample);
        kf = kalmanfilter(kf);
        xh(:,sample) = kf.x;
    end
    
    dx_kf = xh(n_joints+1:2*n_joints,:);
    ddx_kf = xh(2*n_joints+1:end,:);
    
    dataset.ddq = ddx_kf';
else
    dataset.ddq = zeros(size(dataset.dq));
end


end


