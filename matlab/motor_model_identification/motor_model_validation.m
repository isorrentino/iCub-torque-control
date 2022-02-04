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

% Choose the joint from the list
joint = 25;

% How many datasets?
num_datasets = 1;
load_dataset_bool = true;

new_logger_dataset = false;

if new_logger_dataset
    load_dataset_new;
else
    load_dataset_old;
end

%% First model 

x1 = linspace(min(mtr_vel_deg_sec)-500,max(mtr_vel_deg_sec)+500,100);
y1 = linspace(min(mtr_curr),max(mtr_curr),50);

[X1,Y1] = meshgrid(x1,y1);

z1 = parameters{joint}.invKtau * Y1 - parameters{joint}.kbemf * X1;

figure
surf(X1,Y1,z1)

xlabel('motor vel')
ylabel('motor current')
zlabel('joint torque')
title(['Joint ', num2str(joint), ' - \tau = k_{\tau} i - k_{bemf} dq'])

hold on

scatter3(mtr_vel_deg_sec,mtr_curr,joint_trq)

RMSE_kbemf = sqrt(mean((joint_trq - parameters{joint}.invKtau * mtr_curr - parameters{joint}.kbemf * mtr_vel_deg_sec).^2));


%% Second model

x2 = linspace(min(mtr_vel_deg_sec)-500,0,50);
y2 = linspace(min(mtr_curr),max(mtr_curr),50);

[X2,Y2] = meshgrid(x2,y2);

x3 = linspace(0,max(mtr_vel_deg_sec)+500,50);
y3 = linspace(min(mtr_curr),max(mtr_curr),50);

[X3,Y3] = meshgrid(x3,y3);

z2 = parameters{joint}.invKtau * Y2 + parameters{joint}.kc_neg - parameters{joint}.kv_neg * X2;
z3 = parameters{joint}.invKtau * Y3 - parameters{joint}.kc_pos - parameters{joint}.kv_pos * X3;


figure
surf(X2,Y2,z2)
hold on
surf(X3,Y3,z3)
xlabel('motor vel')
ylabel('motor current')
zlabel('joint torque')
title(['Joint ', num2str(joint), ' - \tau = k_{\tau} i - k_{c} sgn(dq) - kv dq'])

scatter3(mtr_vel_deg_sec,mtr_curr,joint_trq)

vel_neg = mtr_vel_deg_sec(mtr_vel_deg_sec <= 0);
curr_vel_neg = mtr_curr(mtr_vel_deg_sec <= 0);
trq_vel_neg = joint_trq(mtr_vel_deg_sec <= 0);
vel_pos = mtr_vel_deg_sec(mtr_vel_deg_sec >= 0);
curr_vel_pos = mtr_curr(mtr_vel_deg_sec >= 0);
trq_vel_pos = joint_trq(mtr_vel_deg_sec >= 0);

yhat_vel_neg = parameters{joint}.invKtau*curr_vel_neg + parameters{joint}.kc_neg - parameters{joint}.kv_neg*vel_neg;
yhat_vel_pos = parameters{joint}.invKtau*curr_vel_pos - parameters{joint}.kc_pos - parameters{joint}.kv_pos*vel_pos;

RMSE_kckv = sqrt(mean(([trq_vel_neg - yhat_vel_neg; trq_vel_pos - yhat_vel_pos]).^2));
