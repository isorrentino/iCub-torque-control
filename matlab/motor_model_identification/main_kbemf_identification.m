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
joint = 25;

% How many datasets?
num_datasets = 1;

load_dataset;

% Identify kbemf (im = 0)
threshold_curr = 0.1;

kbemf = -mtr_vel_deg_sec(abs(mtr_curr) < threshold_curr) \ joint_trq(abs(mtr_curr) < threshold_curr);

figure,
scatter(mtr_vel_deg_sec(abs(mtr_curr) < threshold_curr),joint_trq(abs(mtr_curr) < threshold_curr))
hold on
scatter(mtr_vel_deg_sec(abs(mtr_curr) < threshold_curr),-kbemf*mtr_vel_deg_sec(abs(mtr_curr) < threshold_curr))
xlabel('motor velocity')
ylabel('joint torque')
legend('measured','estimated')
title(Joint_state.joints{joint})

idx_neg = find(abs(mtr_curr) < threshold_curr & mtr_vel_deg_sec<=0);
idx_pos = find(abs(mtr_curr) < threshold_curr & mtr_vel_deg_sec>=0);

reg1 = [sign(mtr_vel_deg_sec(idx_neg)), mtr_vel_deg_sec(idx_neg)];
k1 = -reg1 \ joint_trq(idx_neg);

reg2 = [sign(mtr_vel_deg_sec(idx_pos)), mtr_vel_deg_sec(idx_pos)];
k2 = -reg2 \ joint_trq(idx_pos);

figure,
scatter(mtr_vel_deg_sec(abs(mtr_curr) < threshold_curr),joint_trq(abs(mtr_curr) < threshold_curr))
hold on
scatter(mtr_vel_deg_sec(idx_neg),-k1(1)*sign(mtr_vel_deg_sec(idx_neg)) - k1(2)*mtr_vel_deg_sec(idx_neg))
scatter(mtr_vel_deg_sec(idx_pos),-k2(1)*sign(mtr_vel_deg_sec(idx_pos)) - k2(2)*mtr_vel_deg_sec(idx_pos))
xlabel('motor velocity')
ylabel('joint torque')
legend('measured','estimated')
title(Joint_state.joints{joint})

