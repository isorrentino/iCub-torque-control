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

% Load parameters
% load('ktau_list.mat');
load('ktau_given_friction_list.mat');
load('kckv_list.mat');
% load('kbemf_list.mat');
% load('ktau_kbemf.mat');
% load('ktau_kckv.mat')

% Choose the joint from the list
joint = 21;

% How many datasets?
num_datasets = 1;
load_dataset_bool = true;

new_logger_dataset = true;

if new_logger_dataset
    load_dataset_new;
else
    load_dataset_old;
end

% Available friction models:
% - linear              =>     tau_f = kbemf dq
% - coulomb_viscous     =>     tau_f = kc sgn(dq) + kv dq
friction_model = 'coulomb_viscous';
ktau_depends_on_friction = true;

num_samples = length(mtr_curr_mA) - 10;


if strcmp(friction_model,'linear')
    %% First model
    
    x1 = linspace(min(mtr_vel_deg_sec)-500,max(mtr_vel_deg_sec)+500,100);
    y1 = linspace(min(mtr_curr_mA),max(mtr_curr_mA),50);
    
    [X1,Y1] = meshgrid(x1,y1);
    
    z1 = ktau_list{joint}.invktau * Y1 - friction_params_list{joint}.kbemf * X1;
    
    figure
    surf(X1,Y1,z1)
    xlabel('motor vel')
    ylabel('motor current')
    zlabel('joint torque')
    title(['Joint ', num2str(joint), ' - \tau = k_{\tau} i - k_{bemf} dq'])
    hold on
    scatter3(mtr_vel_deg_sec(1:num_samples),mtr_curr_mA(1:num_samples),joint_trq(1:num_samples),10)
    
    
    estim_joint_trq = ktau_list{joint}.invktau * mtr_curr_mA(1:num_samples) - friction_params_list{joint}.kbemf * mtr_vel_deg_sec(1:num_samples);
    
    figure,
    plot(0:num_samples-1, joint_trq(1:num_samples))
    hold on
    plot(0:num_samples-1, estim_joint_trq(1:num_samples))
    xlabel('samples')
    ylabel('\tau')
    title(['Joint ', num2str(joint), ' - \tau = k_{\tau} i - k_{bemf} dq'])
    legend('measured','estimated')
    
    figure,
    plot(0:num_samples-1, joint_trq(1:num_samples)-estim_joint_trq(1:num_samples))
    xlabel('samples')
    ylabel('\tau error')
    title(['Joint ', num2str(joint), ' - \tau = k_{\tau} i - k_{bemf} dq'])
    legend('error')
    
    
    disp('RMSE - linear friction model')
    RMSE_kbemf = sqrt(mean((joint_trq(1:num_samples) - ktau_list{joint}.invktau * mtr_curr_mA(1:num_samples) - friction_params_list{joint}.kbemf * mtr_vel_deg_sec(1:num_samples)).^2));
    disp(RMSE_kbemf)
    
elseif strcmp(friction_model,'coulomb_viscous')
    %% Second model
    
    if ktau_depends_on_friction
        
        x2 = linspace(min(mtr_vel_deg_sec)-500,0,50);
        y2 = linspace(min(mtr_curr_mA),max(mtr_curr_mA),50);
        
        [X2,Y2] = meshgrid(x2,y2);
        
        x3 = linspace(0,max(mtr_vel_deg_sec)+500,50);
        y3 = linspace(min(mtr_curr_mA),max(mtr_curr_mA),50);
        
        [X3,Y3] = meshgrid(x3,y3);
        
        z2 = ktau_list{joint}.invktau_vel_neg * Y2 + friction_params_list{joint}.kc_neg - friction_params_list{joint}.kv_neg * X2;
        z3 = ktau_list{joint}.invktau_vel_pos * Y3 - friction_params_list{joint}.kc_pos - friction_params_list{joint}.kv_pos * X3;
        
        
        estim_joint_trq = zeros(length(joint_trq),1);
        for i = 1 : num_samples
            if mtr_vel_deg_sec(i) >= 0
                estim_joint_trq(i) = ktau_list{joint}.invktau_vel_neg*mtr_curr_mA(i) - friction_params_list{joint}.kc_pos - friction_params_list{joint}.kv_pos*mtr_vel_deg_sec(i);
            else
                estim_joint_trq(i) = ktau_list{joint}.invktau_vel_pos*mtr_curr_mA(i) + friction_params_list{joint}.kc_neg - friction_params_list{joint}.kv_neg*mtr_vel_deg_sec(i);
            end
        end
        
        figure,
        plot(0:num_samples-1, joint_trq(1:num_samples))
        hold on
        plot(0:num_samples-1, estim_joint_trq(1:num_samples))
        xlabel('samples')
        ylabel('\tau')
        title(['Joint ', num2str(joint), '- \tau = k_{\tau} i - k_{c} sgn(dq) - kv dq'])
        legend('measured','estimated')
        
        figure,
        plot(0:num_samples-1, joint_trq(1:num_samples)-estim_joint_trq(1:num_samples))
        xlabel('samples')
        ylabel('\tau error')
        title(['Joint ', num2str(joint), '- \tau = k_{\tau} i - k_{c} sgn(dq) - kv dq'])
        legend('error')
        
        figure
        surf(X2,Y2,z2)
        hold on
        surf(X3,Y3,z3)
        xlabel('motor vel')
        ylabel('motor current')
        zlabel('joint torque')
        title(['Joint ', num2str(joint), ' - \tau = k_{\tau} i - k_{c} sgn(dq) - kv dq'])
        scatter3(mtr_vel_deg_sec(1:num_samples),mtr_curr_mA(1:num_samples),joint_trq(1:num_samples),10)
        
        vel_neg = mtr_vel_deg_sec(mtr_vel_deg_sec(1:num_samples) <= 0);
        curr_vel_neg = mtr_curr_mA(mtr_vel_deg_sec(1:num_samples) <= 0);
        trq_vel_neg = joint_trq(mtr_vel_deg_sec(1:num_samples) <= 0);
        vel_pos = mtr_vel_deg_sec(mtr_vel_deg_sec(1:num_samples) >= 0);
        curr_vel_pos = mtr_curr_mA(mtr_vel_deg_sec(1:num_samples) >= 0);
        trq_vel_pos = joint_trq(mtr_vel_deg_sec(1:num_samples) >= 0);
        
        yhat_vel_neg = ktau_list{joint}.invktau_vel_neg*curr_vel_neg + friction_params_list{joint}.kc_neg - friction_params_list{joint}.kv_neg*vel_neg;
        yhat_vel_pos = ktau_list{joint}.invktau_vel_pos*curr_vel_pos - friction_params_list{joint}.kc_pos - friction_params_list{joint}.kv_pos*vel_pos;
        
        disp('RMSE - c+v friction model')
        RMSE_kckv = sqrt(mean((joint_trq(1:num_samples) - estim_joint_trq(1:num_samples)).^2));
        disp(RMSE_kckv)
        
    else
        
        x2 = linspace(min(mtr_vel_deg_sec)-500,0,50);
        y2 = linspace(min(mtr_curr_mA),max(mtr_curr_mA),50);
        
        [X2,Y2] = meshgrid(x2,y2);
        
        x3 = linspace(0,max(mtr_vel_deg_sec)+500,50);
        y3 = linspace(min(mtr_curr_mA),max(mtr_curr_mA),50);
        
        [X3,Y3] = meshgrid(x3,y3);
        
        z2 = ktau_list{joint}.invktau* Y2 + friction_params_list{joint}.kc_neg - friction_params_list{joint}.kv_neg * X2;
        z3 = ktau_list{joint}.invktau* Y3 - friction_params_list{joint}.kc_pos - friction_params_list{joint}.kv_pos * X3;
        
        
        estim_joint_trq = zeros(length(joint_trq),1);
        for i = 1 : num_samples
            if mtr_vel_deg_sec(i) >= 0
                estim_joint_trq(i) = ktau_list{joint}.invktau*mtr_curr_mA(i) - friction_params_list{joint}.kc_pos - friction_params_list{joint}.kv_pos*mtr_vel_deg_sec(i);
            else
                estim_joint_trq(i) = ktau_list{joint}.invktau*mtr_curr_mA(i) + friction_params_list{joint}.kc_neg - friction_params_list{joint}.kv_neg*mtr_vel_deg_sec(i);
            end
        end
        
        figure,
        plot(0:num_samples-1, joint_trq(1:num_samples))
        hold on
        plot(0:num_samples-1, estim_joint_trq(1:num_samples))
        xlabel('samples')
        ylabel('\tau')
        title(['Joint ', num2str(joint), '- \tau = k_{\tau} i - k_{c} sgn(dq) - kv dq'])
        legend('measured','estimated')
        
        figure,
        plot(0:num_samples-1, joint_trq(1:num_samples)-estim_joint_trq(1:num_samples))
        xlabel('samples')
        ylabel('\tau error')
        title(['Joint ', num2str(joint), '- \tau = k_{\tau} i - k_{c} sgn(dq) - kv dq'])
        legend('error')
        
        figure
        surf(X2,Y2,z2)
        hold on
        surf(X3,Y3,z3)
        xlabel('motor vel')
        ylabel('motor current')
        zlabel('joint torque')
        title(['Joint ', num2str(joint), ' - \tau = k_{\tau} i - k_{c} sgn(dq) - kv dq'])
        scatter3(mtr_vel_deg_sec(1:num_samples),mtr_curr_mA(1:num_samples),joint_trq(1:num_samples),10)
        
        vel_neg = mtr_vel_deg_sec(mtr_vel_deg_sec(1:num_samples) <= 0);
        curr_vel_neg = mtr_curr_mA(mtr_vel_deg_sec(1:num_samples) <= 0);
        trq_vel_neg = joint_trq(mtr_vel_deg_sec(1:num_samples) <= 0);
        vel_pos = mtr_vel_deg_sec(mtr_vel_deg_sec(1:num_samples) >= 0);
        curr_vel_pos = mtr_curr_mA(mtr_vel_deg_sec(1:num_samples) >= 0);
        trq_vel_pos = joint_trq(mtr_vel_deg_sec(1:num_samples) >= 0);
        
        yhat_vel_neg = ktau_list{joint}.invktau*curr_vel_neg + friction_params_list{joint}.kc_neg - friction_params_list{joint}.kv_neg*vel_neg;
        yhat_vel_pos = ktau_list{joint}.invktau*curr_vel_pos - friction_params_list{joint}.kc_pos - friction_params_list{joint}.kv_pos*vel_pos;
        
        disp('RMSE - c+v friction model')
        RMSE_kckv = sqrt(mean((joint_trq(1:num_samples) - estim_joint_trq(1:num_samples)).^2));
        disp(RMSE_kckv)
        
    end
    
end


