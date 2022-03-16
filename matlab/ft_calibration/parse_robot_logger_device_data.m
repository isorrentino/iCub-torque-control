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




% % convertion to radians
% deg2rad = pi/180.0;
% 
% 
% % Load ft sensors data
% 
% ft_data_file_name = strcat(config.yarp_port_data_type,'/data.log');
% 
% ft_data_dir = strcat('./data/',config.experiment,'/',config.ft_robot_part_name,'/',ft_data_file_name);
% 
% % ftPortType = 'forceTorque';
% 
% [dataset.ft_data, dataset.time] = readDataDumper(ft_data_dir);
% 
% 
% 
% % Load joint state data
% 
% joint_state_data_file = strcat(config.yarp_state_port_name,'/data.log');
% 
% robot_parts = fieldnames(config.part_names_values);
% 
% for i = 1:size(robot_parts,1)
%     
%     joint_state_data_dirs{i} = strcat('./data/',config.experiment,'/',robot_parts{i},'/',joint_state_data_file);
%     
% end
% 
% 
% 
% %%%%%%%%%%%%
% dofs = estimator.model().getNrOfDOFs();
% for i=0:dofs-1
%     joint_names{i+1} = estimator.model().getJointName(i);
% end
% %%%%%%%%%%%%
% 
% for i = 1:size(robot_parts)
%     
%     n_joints = size(robot_parts.(robot_parts{i}));
%     
%     [q_temp, dq_temp, ddq_temp, time_temp, qm_temp, dqm_temp, ddqm_temp, tau_temp, pwm_temp] = readStateExt(n_joints, joint_state_data_dirs{i});
%     
%     for j = 1 : n_joints
%         
%         index = find(strcmp(joint_names, robot_parts.(robot_parts{i}){j}));
%         
%         if(isempty(index)==0)
%             dataset.jointNames{index} = robot_parts.(robot_parts{i}){j};
%             
%             q_all(:,index)   =  deg2rad * q_temp(:,j);
%             dq_all(:,index)  =  deg2rad * dq_temp(:,j);
%             ddq_all(:,index) =  deg2rad * ddq_temp(:,j);
%             tau_all(:,index)  =  tau_temp(:,j);
%         end
%         
%     end
%     
% end




end


