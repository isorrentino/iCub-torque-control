load_dataset_bool = true;
if load_dataset_bool
    cd '/home/isorrentino/dev/Datasets/YarpRobotLogger';
    [file,path] = uigetfile('*.mat','Select dataset for identification');
    matToLoad = [path, file];
    load(matToLoad);
    cd '/home/isorrentino/dev/iCub-torque-control/matlab/motor_model_identification';
end

joint_trq = Joint_state.joint_torques(joint,:)';
mtr_curr = Motor_state.motor_currents(joint,:)';
mtr_vel = Motor_state.motor_velocities(joint,:)' * pi/180;