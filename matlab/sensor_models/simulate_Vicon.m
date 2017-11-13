function [quat, pos] = simulate_Vicon()
%% Read data from MAT files
load('ang_vel_save.mat');
load('lin_ang_accel_save.mat');
load('quat_save.mat');
load('lin_vel.mat');
load('time.mat');
load('x_pos.mat');
load('y_pos.mat');
load('z_pos.mat');

quat_init = [0 0 0 1];
pos_init = [1, 0, 0.2];
v_init = [0,0,0];


W(:,1) = [quat_init';pos_init';v_init'];

for i = 1:1:length(ang_vel_save)
    tspan = [t(i) t(i+1)];
    [t_soln, W_soln] = ode45(@(t, W) ViconModel(t, W, ang_vel_save(i,:)', lin_ang_accel_save(i,1:3)'), tspan, W(:,i));
    W(:,i+1) =  W_soln(end,1:10)';
end   
pos = W(5:7,:);
%% Initialize IMU data
quat.x(:,1) = W(1,:);
quat.y(:,1) = W(2,:);
quat.z(:,1) = W(3,:);
quat.w(:,1) = W(4,:);
% pos = [x_pos(1:length(t)-1), y_pos(1:length(t)-1), z_pos(1:length(t)-1)];
end