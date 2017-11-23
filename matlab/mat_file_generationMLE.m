%% Clear workspace
clc; clear all; close all;

%% Add relevant paths
addpath('sensor_models');
addpath('classes');
addpath('utilities');
addpath('controllers');
addpath('solvers');
addpath('trajectory_generator');

%% Simulation for theta_actual
actualTheta = 1;            % Running simulation with actual_theta
quad_dyn_main(actualTheta);
clc; close all; clear all;
%% IMU Data
[a, a_v, omega, QuadrotorModel] = simulate_IMU(QuadrotorModel);
save('../matlab/transform_estimator/Mat_file_logs/U_IMU_accel.mat', 'a');
save('../matlab/transform_estimator/Mat_file_logs/U_IMU_omega.mat', 'omega');

%% Simulation for theta_bar
actualTheta = 0;            % Running simulation with theta_bar
quad_dyn_main(actualTheta);
close all;

%% Load generated matfiles as matrices
load('../matlab/transform_estimator/Mat_file_logs/theta_actual.mat');
load('../matlab/transform_estimator/Mat_file_logs/theta_bar.mat');
load('../matlab/transform_estimator/Mat_file_logs/X_bar.mat');
load('../matlab/transform_estimator/Mat_file_logs/measurement_Z.mat');
load('../matlab/transform_estimator/Mat_file_logs/U_rpm_MAV.mat');
load('../matlab/transform_estimator/Mat_file_logs/U_IMU_accel.mat');
load('../matlab/transform_estimator/Mat_file_logs/U_IMU_omega.mat');

%% Delete all the matfiles
delete('../matlab/transform_estimator/Mat_file_logs/theta_actual.mat');
delete('../matlab/transform_estimator/Mat_file_logs/theta_bar.mat');
delete('../matlab/transform_estimator/Mat_file_logs/X_bar.mat');
delete('../matlab/transform_estimator/Mat_file_logs/measurement_Z.mat');
delete('../matlab/transform_estimator/Mat_file_logs/U_rpm_MAV.mat');
delete('../matlab/transform_estimator/Mat_file_logs/U_IMU_accel.mat');
delete('../matlab/transform_estimator/Mat_file_logs/U_IMU_omega.mat');

%% Create the final structure
fin_vect.theta_actual = theta_actual;
fin_vect.theta_bar = theta_bar;
fin_vect.X_bar = new_W;
fin_vect.Z = measurement_Z;
fin_vect.U_rpm = rpm;
fin_vect.U_IMU_accel = a;
fin_vect.U_IMU_omega = omega;
%% Save the final vector
save('../matlab/transform_estimator/Mat_file_logs/final_Struct.mat', 'fin_vect');