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
joint = 22;

% How many datasets?
num_datasets = 3;

new_logger_dataset = true;

if new_logger_dataset
    load_dataset_new;
else
    load_dataset_old;
end

% Identify ktau^-1 (dot theta = 0)
threshold_vel = 0.05;

% The motor current is logged in Ampere, while the k_tau considers the
% current in mA (that's why we need to multiply times 1000)
invKtau = mtr_curr_mA(abs(mtr_vel_rad_sec) < threshold_vel) \ joint_trq(abs(mtr_vel_rad_sec) < threshold_vel);
Ktau = 1 / invKtau;

figure,
scatter(mtr_curr_mA(abs(mtr_vel_rad_sec) < threshold_vel),joint_trq(abs(mtr_vel_rad_sec) < threshold_vel))
hold on
scatter(mtr_curr_mA(abs(mtr_vel_rad_sec) < threshold_vel),invKtau*mtr_curr_mA(abs(mtr_vel_rad_sec) < threshold_vel))
xlabel('motor current')
ylabel('joint torque')
legend('measured','estimated')
if new_logger_dataset
    title(robot_logger_device.description_list{joint},'Interpreter','none')
else
    title(Joint_state.joints{joint},'Interpreter','none')
end


load('ktau_list.mat');
ktau_list{joint}.invktau = invKtau;
ktau_list{joint}.ktau = Ktau;
save('ktau_list.mat','ktau_list');


