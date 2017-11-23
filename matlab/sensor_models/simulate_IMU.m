function [a, a_v, omega, QuadModel] = simulate_IMU(QuadModel)
%% Read data from MAT files
load('../matlab/sensor_models/DataLog/ang_vel_save.mat');
load('../matlab/sensor_models/DataLog/lin_ang_accel_save.mat');
load('../matlab/sensor_models/DataLog/quat_save.mat');
load('../matlab/sensor_models/DataLog/time.mat');
load('../matlab/sensor_models/DataLog/x_pos.mat');
load('../matlab/sensor_models/DataLog/y_pos.mat');
load('../matlab/sensor_models/DataLog/z_pos.mat');

%% Initialize IMU data
a = zeros(length(t)-1, 3);
omega = zeros(length(t)-1, 3);

%% Set actual value IMU Vicon Transform
QuadModel.IMUViconTransform.quat.x = 0;%0.0454;%0.0872; %10 degrees in roll
QuadModel.IMUViconTransform.quat.y = 0;%-0.0416; %10
QuadModel.IMUViconTransform.quat.z = 0;%0.0454;%0.6428;%0.0872;
QuadModel.IMUViconTransform.quat.w = 1;%0.9971;%0.7660;%0.9962;

QuadModel.IMUViconTransform.pos = [0.0; 0.0; 0.0]; %[0.0726; 0.012; 0.018];

%% Simulation loop
  for i = 1:1:(length(t)-1)
    % Assignments for current state
    QuadModel.currentQuaternion.x = quat_save(i,1);
    QuadModel.currentQuaternion.y = quat_save(i,2);
    QuadModel.currentQuaternion.z = quat_save(i,3);
    QuadModel.currentQuaternion.w = quat_save(i,4);
    QuadModel.currentPosition = [x_pos(i); y_pos(i); z_pos(i)];
    
    QuadModel.currentAngVel = ang_vel_save(i, 1:3)';
    QuadModel.currentAngAcc = lin_ang_accel_save(i, 4:6)';
    QuadModel = QuadModel.setcurrentLinAcc(lin_ang_accel_save(i, 1:3)');
    
    R = QuatToR(QuadModel.currentQuaternion);
    a_v(i,:) = R*lin_ang_accel_save(i,1:3)';
    
    % Generate IMU data
    [a(i,:), omega(i,:)] = QuadModel.genIMUData();
    [a(i,:), omega(i,:)] = sensor_noise(a(i,:)', omega(i,:)');
  end
end
