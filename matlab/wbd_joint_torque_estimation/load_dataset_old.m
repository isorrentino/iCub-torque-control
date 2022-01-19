if load_dataset_bool
    for ii = 1 : num_datasets
        [file,path] = uigetfile('*.mat','Select dataset for identification');
        matToLoad = [path, file];
        load(matToLoad);
        
        if ~exist('joint_pos','var')
            joint_pos = [];
        end
        joint_pos = [joint_pos;Joint_state.joint_positions(joint,:)];
        
        if ~exist('joint_vel','var')
            joint_vel = [];
        end
        joint_vel = [joint_vel;Joint_state.joint_velocities(joint,:)];
        
        if ~exist('joint_trq','var')
            joint_trq = [];
        end
        joint_trq = [joint_trq;Joint_state.joint_torques(joint,:)];
        
        if ~exist('right_front_wrench_client','var')
            right_front_wrench = [];
        end
        right_front_wrench = [right_front_wrench;CartesianWrench.right_front_wrench_client];
        
        if ~exist('right_rear_wrench_client','var')
            right_rear_wrench = [];
        end
        right_rear_wrench = [right_rear_wrench;CartesianWrench.right_rear_wrench_client];
        
        
        if ~exist('right_rear_ft_client','var')
            right_rear_ft = [];
        end
        right_rear_ft = [right_rear_ft;FT.right_rear_ft_client];
        
        if ~exist('right_front_ft_client','var')
            right_front_ft = [];
        end
        right_front_ft = [right_front_ft;FT.right_front_ft_client];
        
        if ~exist('timestamp','var')
            timestamp = [];
        end
        timestamp = [timestamp;time];
    end
end