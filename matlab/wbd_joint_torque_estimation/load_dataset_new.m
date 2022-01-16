if load_dataset_bool
    for ii = 1 : num_datasets
        
        [file,path] = uigetfile('*.mat','Select dataset for identification');
        matToLoad = [path, file];
        load(matToLoad);
        
        
        if ~exist('joint_pos','var')
            joint_pos = [];
        end
        joint_pos(:,:) = [joint_pos;robot_logger_device.joints_state.positions.data(joint,1,:)];
        
        if ~exist('joint_vel','var')
            joint_vel = [];
        end
        joint_vel(:,:) = [joint_vel;robot_logger_device.joints_state.velocities.data(joint,1,:)];
        
        if ~exist('joint_trq','var')
            joint_trq = [];
        end
        joint_trq(:,:) = [joint_trq;robot_logger_device.joints_state.torques.data(joint,1,:)];
        
        if ~exist('right_front_wrench_client','var')
            right_front_wrench_client = [];
        end
        right_front_wrench_client(:,:) = [right_front_wrench_client;robot_logger_device.cartesian_wrenches.right_front_wrench_client.data];
        
        if ~exist('right_rear_wrench_client','var')
            right_rear_wrench_client = [];
        end
        right_rear_wrench_client(:,:) = [right_rear_wrench_client;robot_logger_device.cartesian_wrenches.right_rear_wrench_client.data];
        
        
        if ~exist('right_front_ft_client','var')
            right_front_ft_client = [];
        end
        right_front_ft_client(:,:) = [right_front_ft_client;robot_logger_device.FTs.right_front_ft_client.data];
        
        if ~exist('right_rear_ft_client','var')
            right_rear_ft_client = [];
        end
        right_rear_ft_client(:,:) = [right_rear_ft_client;robot_logger_device.FTs.right_rear_ft_client.data];
        
        if ~exist('time','var')
            time = [];
        end
        time = [time;robot_logger_device.joints_state.positions.timestamps];
    end
end
