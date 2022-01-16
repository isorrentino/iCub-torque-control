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
            right_front_wrench_client = [];
        end
        right_front_wrench_client = [right_front_wrench_client;CartesianWrench.right_front_wrench_client];
        
        if ~exist('right_rear_wrench_client','var')
            right_rear_wrench_client = [];
        end
        right_rear_wrench_client = [right_rear_wrench_client;CartesianWrench.right_rear_wrench_client];
        
        
        if ~exist('right_rear_ft_client','var')
            right_rear_ft_client = [];
        end
        right_rear_ft_client = [right_rear_ft_client;FT.right_rear_ft_client];
        
        if ~exist('right_front_ft_client','var')
            right_front_ft_client = [];
        end
        right_front_ft_client = [right_front_ft_client;FT.right_front_ft_client];
        
    end
end