for ii = 1 : num_datasets
    if load_dataset_bool
        [file,path] = uigetfile('*.mat','Select dataset for identification');
        matToLoad = [path, file];
        load(matToLoad);
    end
    
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
    
    if strcmp(leg,'right')
        if ~exist('right_foot_wrench','var')
            right_foot_wrench = [];
        end
        right_foot_wrench = [right_foot_wrench;CartesianWrench.right_foot_wrench_client];
    else
        if ~exist('left_foot_wrench','var')
            left_foot_wrench = [];
        end
        left_foot_wrench = [left_foot_wrench;CartesianWrench.left_foot_wrench_client];
    end
end
