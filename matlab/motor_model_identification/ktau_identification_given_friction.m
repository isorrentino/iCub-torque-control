%% First run friction_model_identification.m to identify the friction model

% List of joints
%  1  -->  {'neck_pitch'      }
%  2  -->  {'neck_roll'       }
%  3  -->  {'neck_yaw'        }
%  4  -->  {'torso_pitch'     }
%  5  -->  {'torso_roll'      }
%  6  -->  {'torso_yaw'       }
%  7  -->  {'l_shoulder_pitch'}
%  8  -->  {'l_shoulder_roll' }
%  9  -->  {'l_shoulder_yaw'  }
% 10  -->  {'l_elbow'         }
% 11  -->  {'r_shoulder_pitch'}
% 12  -->  {'r_shoulder_roll' }
% 13  -->  {'r_shoulder_yaw'  }
% 14  -->  {'r_elbow'         }
% 15  -->  {'l_hip_pitch'     }
% 16  -->  {'l_hip_roll'      }
% 17  -->  {'l_hip_yaw'       }
% 18  -->  {'l_knee'          }
% 19  -->  {'l_ankle_pitch'   }
% 20  -->  {'l_ankle_roll'    }
% 21  -->  {'r_hip_pitch'     }
% 22  -->  {'r_hip_roll'      }
% 23  -->  {'r_hip_yaw'       }
% 24  -->  {'r_knee'          }
% 25  -->  {'r_ankle_pitch'   }
% 26  -->  {'r_ankle_roll'    }

clear;

% Available friction models:
% - linear              =>     tau_f = kbemf dq
% - coulomb_viscous     =>     tau_f = kc sgn(dq) + kv dq
friction_model = 'coulomb_viscous';

% Choose the joint from the list
joint = 25;

% How many datasets?
num_datasets = 3;

% Load dataset with latest or previous yarp-robot-logger-device version?
logger_new_version = true;

if logger_new_version
    load_dataset_new;
else
    load_dataset_old;
end

threshold_velocity = 1; % deg/sec

if strcmp(friction_model, 'linear')
    
    load('kbemf_list.mat');
    
    % Ax = b
    % current * ktau = tau - tau_f
    
    A = mtr_curr_mA;
    b = joint_trq + friction_params_list{joint}.kbemf * mtr_vel_deg_sec;
    disp('Conditioning regressor')
    cond(A)
    invktau = A \ b;
    ktau = 1 / invktau;
    
    ktau_list{joint}.invktau = invktau;
    ktau_list{joint}.ktau = ktau;
    save('ktau_given_friction_list.mat','ktau_list');
    
elseif strcmp(friction_model, 'coulomb_viscous')
    
    load('kckv_list.mat');
    
    % Ax = b
    % current * ktau = tau - tau_f
    
    current_vel_neg = mtr_curr_mA(mtr_vel_deg_sec <= -threshold_velocity & joint_trq < 30);
    current_vel_pos = mtr_curr_mA(mtr_vel_deg_sec >= threshold_velocity & joint_trq > -30);
    mtr_vel_neg = mtr_vel_deg_sec(mtr_vel_deg_sec <= -threshold_velocity & joint_trq < 30);
    mtr_vel_pos = mtr_vel_deg_sec(mtr_vel_deg_sec >= threshold_velocity & joint_trq > -30);
    trq_vel_neg = joint_trq(mtr_vel_deg_sec <= -threshold_velocity & joint_trq < 30);
    trq_vel_pos = joint_trq(mtr_vel_deg_sec >= threshold_velocity & joint_trq > -30);
    
    A_neg = current_vel_neg;
    b_vel_neg = trq_vel_neg - friction_params_list{joint}.kc_neg + friction_params_list{joint}.kv_neg * mtr_vel_neg;
    disp('Conditioning regressor - negative part')
    cond(A_neg)
    invktau_vel_neg = A_neg \ b_vel_neg;
    
    A_pos = current_vel_pos;
    b_vel_pos = trq_vel_pos + friction_params_list{joint}.kc_pos + friction_params_list{joint}.kv_pos * mtr_vel_pos;
    disp('Conditioning regressor - positive part')
    cond(A_pos)
    invktau_vel_pos = A_pos \ b_vel_pos;
    
    ktau_neg = 1/invktau_vel_neg;
    ktau_pos = 1/invktau_vel_pos;
    
    load('ktau_given_friction_list.mat');
    ktau_list{joint}.invktau_vel_neg = invktau_vel_neg;
    ktau_list{joint}.invktau_vel_pos = invktau_vel_pos;
    ktau_list{joint}.ktau_neg = ktau_neg;
    ktau_list{joint}.ktau_pos = ktau_pos;
    save('ktau_given_friction_list.mat','ktau_list');
    
    figure,
    scatter(mtr_curr_mA, joint_trq)
    hold on
    scatter([current_vel_neg; current_vel_pos], [invktau_vel_neg* ...
        current_vel_neg+friction_params_list{joint}.kc_neg-friction_params_list{joint}.kv_neg*mtr_vel_neg; ...
        invktau_vel_pos*current_vel_pos-friction_params_list{joint}.kc_pos-friction_params_list{joint}.kv_pos*mtr_vel_pos])
    xlabel('motor current')
    ylabel('joint torque')
    legend('measured','estimated')
    if logger_new_version
        title(robot_logger_device.description_list{joint},'Interpreter','none')
    else
        title(Joint_state.joints{joint},'Interpreter','none')
    end
    
end




