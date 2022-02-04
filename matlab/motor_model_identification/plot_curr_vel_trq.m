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

load('parameters.mat');

plot_separate_experiments = true;

% Choose the joint from the list
joint = 26;

% How many datasets?
num_datasets = 1;
load_dataset_bool = true;

new_logger_dataset = false;

if plot_separate_experiments
    
    new_logger_dataset = false;
    
    if new_logger_dataset
        load_dataset_new;
    else
        load_dataset_old;
    end
    
    mtr_vel_deg_sec1 = mtr_vel_deg_sec;
    mtr_curr1 = mtr_curr;
    joint_trq1 = joint_trq;
    
    threshold_curr = 0.05;
    
    figure;
    s1 = subplot(1,2,1);
    scatter3(mtr_vel_deg_sec1(abs(mtr_curr1) < threshold_curr),mtr_curr1(abs(mtr_curr1) < threshold_curr),joint_trq1(abs(mtr_curr1) < threshold_curr));
    hold on
    xlabel('motor vel')
    ylabel('motor current')
    zlabel('joint torque')
    
    
    if new_logger_dataset
        load_dataset_new;
    else
        load_dataset_old;
    end
    
    mtr_vel_deg_sec2 = mtr_vel_deg_sec;
    mtr_curr2 = mtr_curr;
    joint_trq2 = joint_trq;
    
    threshold_vel = 0.05;
    
    scatter3(mtr_vel_deg_sec2(abs(mtr_vel_deg_sec2) < threshold_vel),mtr_curr2(abs(mtr_vel_deg_sec2) < threshold_vel),joint_trq2(abs(mtr_vel_deg_sec2) < threshold_vel));
    
    legend('\tau = k_{bemf} dq','\tau = k_{\tau} i')
    
    
    
    
    new_logger_dataset = true;
    
    if new_logger_dataset
        load_dataset_new;
    else
        load_dataset_old;
    end
    
    mtr_vel_deg_sec3 = mtr_vel_deg_sec;
    mtr_curr3 = mtr_curr;
    joint_trq3 = joint_trq;
    
    subplot(1,2,2)
    s2 = scatter3(mtr_vel_deg_sec3,mtr_curr3,joint_trq3);
    xlabel('motor vel')
    ylabel('motor current')
    zlabel('joint torque')
    title('\tau = k_{\tau} i + k_{bemf} dq')
    hold on 
    scatter3(mtr_vel_deg_sec1(abs(mtr_curr1) < threshold_curr),mtr_curr1(abs(mtr_curr1) < threshold_curr),joint_trq1(abs(mtr_curr1) < threshold_curr));
    scatter3(mtr_vel_deg_sec2(abs(mtr_vel_deg_sec2) < threshold_vel),mtr_curr2(abs(mtr_vel_deg_sec2) < threshold_vel),joint_trq2(abs(mtr_vel_deg_sec2) < threshold_vel));
    
%     estim_joint_trq = parameters{joint}.invKtau * mtr_curr3 - parameters{joint}.kbemf * mtr_vel_deg_sec3;
    estim_joint_trq = 1000/-1.0294e+04 * mtr_curr3 - (-1.6377e-04) * mtr_vel_deg_sec3;
    scatter3(mtr_vel_deg_sec3,mtr_curr3,estim_joint_trq);
    
    idx_neg = find(mtr_vel_deg_sec<=0);
    idx_pos = find(mtr_vel_deg_sec>=0);
    
    estim_joint_trq_neg_model = parameters{joint}.invKtau * mtr_curr3(idx_neg) - parameters{joint}.kc_neg * sign(mtr_vel_deg_sec3(idx_neg)) - parameters{joint}.kv_neg * mtr_vel_deg_sec3(idx_neg);
    estim_joint_trq_pos_model = parameters{joint}.invKtau * mtr_curr3(idx_pos) - parameters{joint}.kc_pos * sign(mtr_vel_deg_sec3(idx_pos)) - parameters{joint}.kv_pos * mtr_vel_deg_sec3(idx_pos);
    
    scatter3(mtr_vel_deg_sec3(idx_neg),mtr_curr3(idx_neg),estim_joint_trq_neg_model);
    scatter3(mtr_vel_deg_sec3(idx_pos),mtr_curr3(idx_pos),estim_joint_trq_pos_model);
    
    legend('\tau_{meas} random traj','\tau_{meas} zero current','\tau_{meas} zero vel','\tau_{estim} linear friction','\tau_{meas} c+v friction','\tau_{meas} c+v friction');

else
    
    if new_logger_dataset
        load_dataset_new;
    else
        load_dataset_old;
    end
    
    figure
    scatter3(mtr_vel_rad_sec,mtr_curr,joint_trq)
    xlabel('motor vel')
    ylabel('motor current')
    zlabel('joint torque')
    
end


