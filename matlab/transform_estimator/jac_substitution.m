function [FM, FI, JTheta] = jac_substitution(Net_subs_mat, values_sub)

%% Running Jacobian calculation for MAV Model
load('Jac_MAVModel_sym.mat');
FM = double(subs(F, Net_subs_mat, values_sub));

%% Running Jacobian calculation for IMU Model
load('Jac_IMUModel_sym.mat');
FI = double(subs(F, Net_subs_mat, values_sub));

%% Running Jacobian calculation based on \theta
load('Jac_jTheta_sym.mat');
JTheta = double(subs(F, Net_subs_mat, values_sub));

end