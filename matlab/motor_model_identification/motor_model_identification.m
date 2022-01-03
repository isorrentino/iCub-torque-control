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

% tau = ktau^(-1) * i - Kbemf \dot(theta)

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
title(Joint_state.joints{joint})
