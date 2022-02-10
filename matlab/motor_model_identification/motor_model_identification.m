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

% Choose the joint from the list
joint = 24;

% How many datasets?
num_datasets = 3;
load_dataset_bool = true;

new_logger_dataset = true;

if new_logger_dataset
    load_dataset_new;
else
    load_dataset_old;
end


%% tau = ktau^(-1) * i - Kbemf \dot(theta)

reg = [mtr_curr, -mtr_vel_deg_sec];
k = reg \ joint_trq;

invKtau = k(1);
kbemf = k(2);

Ktau = 1000/invKtau;

figure,
scatter(0:length(joint_trq)-1,joint_trq)
hold on
scatter(0:length(joint_trq)-1, invKtau*mtr_curr - kbemf*mtr_vel_deg_sec)
xlabel('time')
ylabel('joint torque')
legend('measured','estimated')
title(robot_logger_device.description_list{joint},'Interpreter','none')

% load('ktau_kbemf.mat');
ktau_list{joint}.invktau = invKtau;
ktau_list{joint}.ktau = Ktau;
friction_params_list{joint}.kbemf = kbemf;
save('ktau_kbemf.mat','ktau_list','friction_params_list');

% RMSE = sqrt(mean((joint_trq - invKtau*mtr_curr - kbemf*mtr_vel_deg_sec).^2));



%% tau = ktau^(-1) * i - kc sgn(\dot(theta)) - kv \dot(theta)

% Negative velocities
% reg_neg = [mtr_curr, -sgn(-1), -dq];
% reg_neg = [mtr_curr, -sgn(1), -dq];

idx_neg = find(mtr_vel_deg_sec <= 0);
idx_pos = find(mtr_vel_deg_sec >= 0);

reg_neg = [mtr_curr(idx_neg), ones(size(mtr_vel_deg_sec(idx_neg))), -mtr_vel_deg_sec(idx_neg)];
reg_pos = [mtr_curr(idx_pos), -ones(size(mtr_vel_deg_sec(idx_pos))), -mtr_vel_deg_sec(idx_pos)];

k_neg = reg_neg \ joint_trq(idx_neg);
k_pos = reg_pos \ joint_trq(idx_pos);

invKtau_neg = k_neg(1);
Ktau_neg = 1000/invKtau_neg;
kc_neg = k_neg(2);
kv_neg = k_neg(3);

invKtau_pos = k_pos(1);
Ktau_pos = 1000/invKtau_pos;
kc_pos = k_pos(2);
kv_pos = k_pos(3);


figure,
scatter(0:length(joint_trq(idx_neg))-1,joint_trq(idx_neg))
hold on
scatter(0:length(joint_trq(idx_neg))-1, invKtau_neg*mtr_curr(idx_neg) - kc_neg*sign(mtr_vel_deg_sec(idx_neg)) - kv_neg*mtr_vel_deg_sec(idx_neg))
xlabel('time')
ylabel('joint torque')
legend('measured','estimated')
title(strcat(robot_logger_device.description_list{joint},'Interpreter','none',' - negative vel'))

figure,
scatter(0:length(joint_trq(idx_pos))-1,joint_trq(idx_pos))
hold on
scatter(0:length(joint_trq(idx_pos))-1, invKtau_pos*mtr_curr(idx_pos) - kc_pos*sign(mtr_vel_deg_sec(idx_pos)) - kv_pos*mtr_vel_deg_sec(idx_pos))
xlabel('time')
ylabel('joint torque')
legend('measured','estimated')
title(strcat(robot_logger_device.description_list{joint},'Interpreter','none',' - positive vel'))

% load('ktau_kckv.mat');
ktau_list{joint}.invktau_vel_neg = invKtau_neg;
ktau_list{joint}.ktau_neg = Ktau_neg;
ktau_list{joint}.invktau_vel_pos = invKtau_pos;
ktau_list{joint}.ktau_pos = Ktau_pos;
friction_params_list{joint}.kc_neg = kc_neg;
friction_params_list{joint}.kv_neg = kv_neg;
friction_params_list{joint}.kc_pos = kc_pos;
friction_params_list{joint}.kv_pos = kv_pos;
save('ktau_kckv.mat','friction_params_list','ktau_list');


% RMSE = sqrt(mean(([joint_trq(idx_neg);joint_trq(idx_pos)] - [invKtau_neg*mtr_curr(idx_neg) + kc_neg*sign(mtr_vel_deg_sec(idx_neg)) - kv_neg*mtr_vel_deg_sec(idx_neg); invKtau_pos*mtr_curr(idx_pos) - kc_pos*sign(mtr_vel_deg_sec(idx_pos)) - kv_pos*mtr_vel_deg_sec(idx_pos)]).^2));



%% Identification with kc and kv already identified


% Negative velocities
% reg_neg = [mtr_curr, -sgn(-1), -dq];
% reg_neg = [mtr_curr, -sgn(1), -dq];

% reg_neg = mtr_curr(idx_neg);
% reg_pos = mtr_curr(idx_pos);

% k_neg_2 = reg_neg \ (joint_trq(idx_neg) + parameters{joint}.kc_neg*sign(mtr_vel_deg_sec(idx_neg)) + parameters{joint}.kv_neg*mtr_vel_deg_sec(idx_neg));
% k_pos_2 = reg_pos \ (joint_trq(idx_pos) + parameters{joint}.kc_pos*sign(mtr_vel_deg_sec(idx_pos)) + parameters{joint}.kv_pos*mtr_vel_deg_sec(idx_pos));

% invKtau_neg_2 = k_neg_2(1);
% Ktau_neg_2 = 1000/invKtau_neg_2;
% kc_neg = k_neg(2);
% kv_neg = k_neg(3);

% invKtau_pos_2 = k_pos_2(1);
% Ktau_pos_2 = 1000/invKtau_pos_2;
% kc_pos = k_pos(2);
% kv_pos = k_pos(3);


% figure,
% scatter(0:length(joint_trq(idx_neg))-1,joint_trq(idx_neg))
% hold on
% scatter(0:length(joint_trq(idx_neg))-1, invKtau_neg_2*mtr_curr(idx_neg) + parameters{joint}.kc_neg*sign(mtr_vel_deg_sec(idx_neg)) - parameters{joint}.kv_neg*mtr_vel_deg_sec(idx_neg))
% xlabel('time')
% ylabel('joint torque')
% legend('measured','estimated')
% title(strcat(Joint_state.joints{joint},' - negative vel'))

% figure,
% scatter(0:length(joint_trq(idx_pos))-1,joint_trq(idx_pos))
% hold on
% scatter(0:length(joint_trq(idx_pos))-1, invKtau_pos_2*mtr_curr(idx_pos) - parameters{joint}.kc_pos*sign(mtr_vel_deg_sec(idx_pos)) - parameters{joint}.kv_pos*mtr_vel_deg_sec(idx_pos))
% xlabel('time')
% ylabel('joint torque')
% legend('measured','estimated')
% title(strcat(Joint_state.joints{joint},' - positive vel'))

% RMSE_2 = sqrt(mean(([joint_trq(idx_neg);joint_trq(idx_pos)] - [invKtau_neg_2*mtr_curr(idx_neg) + parameters{joint}.kc_neg*sign(mtr_vel_deg_sec(idx_neg)) - parameters{joint}.kv_neg*mtr_vel_deg_sec(idx_neg);invKtau_pos_2*mtr_curr(idx_pos) - parameters{joint}.kc_pos*sign(mtr_vel_deg_sec(idx_pos)) - parameters{joint}.kv_pos*mtr_vel_deg_sec(idx_pos)]).^2));



% parameters{joint}.invKtau = invKtau_neg_2;
% parameters{joint}.Ktau = Ktau_neg_2;
% save('parameters_friction.mat','parameters');


