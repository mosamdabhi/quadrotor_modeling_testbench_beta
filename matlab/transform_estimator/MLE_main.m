clear;
clc;
close all;

addpath('../sensor_models/DataLog/');
addpath('../sensor_models/');
addpath('../classes');
addpath('../utilities');
addpath('Jacobian_MatFiles/');
addpath('../');
addpath('../../matlab_utils/src/');
addpath('Mat_file_logs/');

%% Run these Jacobian syms generating scripts ONLY ONCE to genreate the Jacobian in syms
% symbolic_calc_MAVModel;     % Jacobian for MAV Dynamics model 
% symbolic_calc_IMUModel;     % Jacobian for IMU model 
% symbolic_calc_jTheta;       % Jacobian for jTheta model

%% Load time and jacobian data.
load('time.mat');
load('FTotalMTotal.mat');
load('Jac_IMUModel_sym.mat');
load('Jac_jTheta_sym.mat');
load('Jac_MAVModel_sym.mat');
load('final_Struct.mat');

dt = t(2) - t(1);
%% Initialization
% Measurement residuals.
z(1:3,:) = fin_vect.Z(1:3,:); % Measurement vector.
z(4:7,:) = fin_vect.Z(7:10,:); 
rz = zeros(9,length(t));
rzNominal = zeros(9,length(t));

% Process residuals.
xMNominal = fin_vect.X_bar; % State vector - MAV Dynamics.
xINominal = zeros(23,length(t)); % State vector - IMU Dynamics.
xEst(1:3, :) = fin_vect.Z(1:3,:); % Estimated state vector.
xEst(7:10, :) = fin_vect.Z(7:10,:); 
xEst(4:6, :) = zeros(3, length(t));
xEst(11:23, :) = zeros(13, length(t));

delChiM = zeros(22,length(t)); % (minimal) state residual
rm = zeros(22,length(t));
rmNominal = zeros(22,length(t));

chiI = zeros(22,length(t)); % Minimal x - IMU Dynamics
delChiI = zeros(22,length(t)); % (minimal) state residual
ri = zeros(22,length(t));
riNominal = zeros(22,length(t));

% Populate delChi using errstate_compute. Nominal - Estimate
for ii = 1:1:length(t)
    delChiM(:, ii) = errstate_compute(xMNominal(:, ii), xEst(:, ii));
end

% Parameter vector (Initialize via MAT file).
thetaNominal = fin_vect.theta_bar;
thetaEst = fin_vect.theta_bar;
deltheta = zeros(9, 1);

% IMU and RPM data
omega = fin_vect.U_IMU_omega';
a = fin_vect.U_IMU_accel';
desRPM = fin_vect.U_rpm;

for n = 1:1:5
for i = 1:1:length(t)-1
    disp(t(i));
    
    % vars for jacobian calculation
    vars = vertcat(xMNominal(1:13, i), xMNominal(17:23, i), thetaNominal(:, 1), omega(:, i), 0.0274, a(:, i), [0; 0; 0], [0; 0; U(1, i)], U(2:4, i), 0, 0);
    
    % Import Jacobians
    [FM, FI, JTheta] = main_jacCalc(vars);
    FM = vertcat(FM(1:9, :), FM(11:end, :));
    FI = vertcat(FI(1:9, :), FI(11:end, :));
    JTheta = vertcat(JTheta(1:9, :), JTheta(11:end, :));
    
    % Measurement model jacobian
    H = [zeros(1,3), zeros(1,3), zeros(1,3), zeros(1,3), zeros(1,3), zeros(1,3), zeros(1,4);
        zeros(1,3), zeros(1,3), zeros(1,3), zeros(1,3), zeros(1,3), zeros(1,3), zeros(1,4);
        zeros(1,3), zeros(1,3), zeros(1,3), zeros(1,3), zeros(1,3), zeros(1,3), zeros(1,4);
        eye(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,4);
        zeros(3,3),zeros(3,3), eye(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,4)];
    
    distrans = eye(22,22) + FM*dt;
    %distransI = eye(22,22) + FI*dt;
    distransTheta = eye(22,9) + JTheta*dt;
    
    % Residual calculation -> update delChi and delZeta
    rz(:, i+1) = rzNominal(:, i+1) - H*(delChiM(:,i+1));
    rm(:, i+1) = rmNominal(:, i+1) - [distrans, -eye(22, 22), distransTheta]*[delChiM(:,i); delChiM(:,i+1); deltheta];
    %ri(:, i+1) = rmNominal(:, i+1) - [distransI, -eye(22, 22)]*[delChi(:,i); delChi(:,i+1)];
    
    % Maximize log likelihood.
    % Finding the best fit using least squares method
    A = [H, zeros(9,9); distrans, distransTheta];% distransI, zeros(6,6)];  
    R = vertcat(rz(:,i+1), rm(:,i+1)); %, ri(:, i+1));               
    varsHat(:, i, n) = (A' * A) \ (A' * R);
end
end

% Plot residuals

% Plot Likelihood
