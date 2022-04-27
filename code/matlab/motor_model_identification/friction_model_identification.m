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

% Available models:
% - linear              =>     tau_f = kbemf dq
% - coulomb + viscous   =>     tau_f = kc sgn(dq) + kv dq

% Choose the joint from the list
joint = 22;

% How many datasets?
num_datasets = 1;

% Load dataset with latest or previous yarp-robot-logger-device version?
new_logger_version = true;

if new_logger_version
    load_dataset_new;
else
    load_dataset_old;
end

% Identify kbemf (im = 0)
% Threshold on the current (current is in Ampere)
threshold_curr = 100;


%% Linear model

disp('Condition number of regressor')
cond(-mtr_vel_deg_sec(abs(mtr_curr_mA) < threshold_curr))

kbemf = -mtr_vel_deg_sec(abs(mtr_curr_mA) < threshold_curr) \ joint_trq(abs(mtr_curr_mA) < threshold_curr);

figure,
scatter(mtr_vel_deg_sec(abs(mtr_curr_mA) < threshold_curr),joint_trq(abs(mtr_curr_mA) < threshold_curr))
hold on
scatter(mtr_vel_deg_sec(abs(mtr_curr_mA) < threshold_curr),-kbemf*mtr_vel_deg_sec(abs(mtr_curr_mA) < threshold_curr))
xlabel('motor velocity')
ylabel('joint torque')
legend('measured','estimated')
if new_logger_version
    title(robot_logger_device.description_list{joint},'Interpreter','none')
else
    title(Joint_state.joints{joint},'Interpreter','none')
end

load('kbemf_list.mat')
friction_params_list{joint}.kbemf = kbemf;
save('kbemf_list.mat','friction_params_list');


%% Coulomb + Viscous

idx_neg = find(abs(mtr_curr_mA) < threshold_curr & mtr_vel_deg_sec < 500 & joint_trq > 3);
idx_pos = find(abs(mtr_curr_mA) < threshold_curr & mtr_vel_deg_sec > 500 & joint_trq < -3);

reg1 = [sign(mtr_vel_deg_sec(idx_neg)), mtr_vel_deg_sec(idx_neg)];
disp('Condition number of regressor - negative part')
cond(-reg1)
k1 = -reg1 \ joint_trq(idx_neg);

reg2 = [sign(mtr_vel_deg_sec(idx_pos)), mtr_vel_deg_sec(idx_pos)];
disp('Condition number of regressor - positive part')
cond(-reg2)
k2 = -reg2 \ joint_trq(idx_pos);

figure,
scatter(mtr_vel_deg_sec(abs(mtr_curr_mA) < threshold_curr),joint_trq(abs(mtr_curr_mA) < threshold_curr))
hold on
scatter(mtr_vel_deg_sec(idx_neg),-k1(1)*sign(mtr_vel_deg_sec(idx_neg)) - k1(2)*mtr_vel_deg_sec(idx_neg))
scatter(mtr_vel_deg_sec(idx_pos),-k2(1)*sign(mtr_vel_deg_sec(idx_pos)) - k2(2)*mtr_vel_deg_sec(idx_pos))
xlabel('motor velocity')
ylabel('joint torque')
legend('measured','estimated')

if new_logger_version
    title(robot_logger_device.description_list{joint},'Interpreter','none')
else
    title(Joint_state.joints{joint},'Interpreter','none')
end

load('kckv_list.mat')
friction_params_list{joint}.kc_neg = k1(1);
friction_params_list{joint}.kv_neg = k1(2);
friction_params_list{joint}.kc_pos = k2(1);
friction_params_list{joint}.kv_pos = k2(2);
save('kckv_list.mat','friction_params_list');


