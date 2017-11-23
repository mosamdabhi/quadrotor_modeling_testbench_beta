%% Clear workspace
clc; clear all; close all;

%% add path for Tim's matlab_utils
addpath('../../matlab_utils/src/');

%% Constants/syms/Varibales declaration
gravity_I = [0; 0; 9.80665];

syms a_x a_y a_z b_a_x b_a_y b_a_z w_a_x w_a_y w_a_z;
syms omega_x omega_y omega_z b_omega_x b_omega_y b_omega_z w_omega_x w_omega_y w_omega_z;
syms qIB_w qIB_x qIB_y qIB_z; 
syms rIB_I_x rIB_I_y rIB_I_z vB_B_x vB_B_y vB_B_z;
syms rBC_B_x rBC_B_y rBC_B_z;
syms omegaX_B omegaY_B omegaZ_B;
syms n1 n2 n3 n4;
syms c_T c_m c_d w_M w_F jxx jyy jzz;

rIB_I = [rIB_I_x; rIB_I_y; rIB_I_z];
rBC_B = [rBC_B_x; rBC_B_y; rBC_B_z];
omega_B = [omegaX_B; omegaY_B; omegaZ_B];
nTotal = [n1; n2; n3; n4];
aTotalIMU = [a_x; a_y; a_z];
b_a_Total = [b_a_x; b_a_y; b_a_z];
w_a_Total = [w_a_x; w_a_y; w_a_z];
omegaTotalIMU = [omega_x; omega_y; omega_z];
b_omega_Total = [b_omega_x; b_omega_y; b_omega_z];
w_omega_Total = [w_omega_x; w_omega_y; w_omega_z];
qIB = [qIB_x; qIB_y; qIB_z; qIB_w];
j_C = [jxx; jyy; jzz];
omega_B_dot = [0; 0; 0];
nTotal_dot = [0; 0; 0; 0];

% Minimal vector calculation.
prompt = 'Value of qIB_w : ';
act_val_qIB_w = input(prompt);
min_q_temp = [qIB_x; qIB_y; qIB_z];
new_qIB = subs(sign(qIB_w)*min_q_temp,qIB_w,sign(act_val_qIB_w));

vB_B = [vB_B_x; vB_B_y; vB_B_z];

CIB = [  1 - 2*qIB_y^2 - 2*qIB_z^2, 2*(qIB_x*qIB_y - qIB_z*qIB_w), 2*(qIB_x*qIB_z + qIB_y*qIB_w);
       2*(qIB_x*qIB_y + qIB_z*qIB_w),   1 - 2*qIB_x^2 - 2*qIB_z^2, 2*(qIB_y*qIB_z - qIB_x*qIB_w);
       2*(qIB_x*qIB_z - qIB_y*qIB_w), 2*(qIB_y*qIB_z + qIB_x*qIB_w),   1 - 2*qIB_x^2 - 2*qIB_y^2];

aTilde = aTotalIMU + b_a_Total + w_a_Total;
omegaTilde = omegaTotalIMU + b_omega_Total + w_omega_Total;

Temp_vBDot_1 = transpose(CIB)*aTilde - b_a_Total - w_a_Total;
Temp_vBDot_2 = gravity_I;
Temp_vBDot_3 = cross((omegaTilde - b_omega_Total - w_omega_Total),vB_B);

omegaSkew = [0 -omega_z omega_y; omega_z 0 -omega_x; -omega_y omega_x 0];

%% f function definition
rIB_I_dot = vB_B;  
vB_B_dot = Temp_vBDot_1 - Temp_vBDot_2;
qIB_dot = [-omegaSkew/2 omegaTotalIMU/2; -transpose(omegaTotalIMU)/2 0]*qIB;
b_omega_Total_dot = w_omega_Total;
b_a_Total_dot = w_a_Total;

%% x_vars calculation
x_vars = [rIB_I; vB_B; qIB; omega_B; b_omega_Total; b_a_Total; nTotal];
minimal_x_vars = [rIB_I; vB_B; new_qIB; omega_B; b_omega_Total; b_a_Total; nTotal];

%% Jacobian Calculation
f = simplify([rIB_I_dot; vB_B_dot; qIB_dot; omega_B_dot; b_omega_Total_dot; b_a_Total_dot; nTotal_dot], 'Steps', 50);
F = simplify(derive_analytic_jacobian(f, minimal_x_vars), 'Steps', 50);

%% Plotting functionality
F_zero = zeros(size(F));
for i = 1:numel(F_zero)
    F_zero(i) = strcmpi(char(F(i)), '0');
end

save ('Jacobian_MatFiles/Jac_IMUModel_sym.mat', 'F');
figure; image(F_zero, 'CDataMapping', 'scaled'); hold on; grid on;