clear;
clc;
close all;

% add path for Tim's matlab_utils
addpath('../../matlab_utils/src');

% define syms for state definition.
syms rIB_I_x rIB_I_y rIB_I_z vB_B_x vB_B_y vB_B_z;
syms rBC_B_x rBC_B_y rBC_B_z;
syms qIB_w qIB_x qIB_y qIB_z; 
syms FTotalX_B FTotalY_B FTotalZ_B;
syms MTotalX_C MTotalY_C MTotalZ_C;
syms omegaX_B omegaY_B omegaZ_B;
syms u1 u2 u3 u4 n1 n2 n3 n4 tau;
syms w_n_Noise_1 w_n_Noise_2 w_n_Noise_3 w_n_Noise_4;
syms jxx jyy jzz;

gravity_I = [0; 0; 9.80665];
mass = 1.251;

rIB_I = [rIB_I_x; rIB_I_y; rIB_I_z];
rBC_B = [rBC_B_x; rBC_B_y; rBC_B_z];
vB_B = [vB_B_x; vB_B_y; vB_B_z];
qIB = [qIB_w; qIB_x; qIB_y; qIB_z];
FTotal_B = [FTotalX_B; FTotalY_B; FTotalZ_B];
MTotal_C = [MTotalX_C; MTotalY_C; MTotalZ_C];
omega_B = [omegaX_B; omegaY_B; omegaZ_B];
nTotal = [n1; n2; n3; n4];
uTotal = [u1; u2; u3; u4];
w_n_NoiseTotal = [w_n_Noise_1; w_n_Noise_2; w_n_Noise_3; w_n_Noise_4];
j_C = [jxx; jyy; jzz];
J_C = diag(j_C);
omegaSkew = [0 -omegaZ_B omegaY_B; omegaZ_B 0 -omegaX_B; -omegaY_B omegaX_B 0];
ohmOmega = [-omegaSkew omega_B; -transpose(omega_B) 0]/2;
x_vars = [rIB_I; vB_B; qIB; omega_B; nTotal];


CIB = [  1 - 2*qIB_y^2 - 2*qIB_z^2, 2*(qIB_x*qIB_y - qIB_z*qIB_w), 2*(qIB_x*qIB_z + qIB_y*qIB_w);
       2*(qIB_x*qIB_y + qIB_z*qIB_w),   1 - 2*qIB_x^2 - 2*qIB_z^2, 2*(qIB_y*qIB_z - qIB_x*qIB_w);
       2*(qIB_x*qIB_z - qIB_y*qIB_w), 2*(qIB_y*qIB_z + qIB_x*qIB_w),   1 - 2*qIB_x^2 - 2*qIB_y^2];

temp_omegaBDot_calc = J_C*omega_B;
omega_B_dot = inv(J_C)*(MTotal_C - cross(omega_B, temp_omegaBDot_calc));
   
Temp_vBDot_1 = (FTotal_B/mass);
Temp_vBDot_2 = transpose(CIB)*gravity_I;
Temp_vBDot_3 = cross(omega_B, vB_B);
Temp_vBDot_4 = cross(omega_B_dot, rBC_B);
Temp_vBDot_5_1 = cross(omega_B, omega_B);
Temp_vBDot_5_2 = cross(Temp_vBDot_5_1, rBC_B);
   
% f function definition
rIB_I_dot = CIB*vB_B;   
vB_B_dot = Temp_vBDot_1 - Temp_vBDot_2 - Temp_vBDot_3 - Temp_vBDot_4 - Temp_vBDot_5_2;
qIB_dot = [-omegaSkew/2 omega_B/2; -transpose(omega_B/2) 0]*qIB;
nTotal_dot = [uTotal - nTotal]/tau + w_n_NoiseTotal;

f = simplify([rIB_I_dot; vB_B_dot; qIB_dot; omega_B_dot; nTotal_dot], 'Steps', 50);
F = simplify(derive_analytic_jacobian(f, x_vars), 'Steps', 50);

F_zero = zeros(size(F));
for i = 1:numel(F_zero)
    F_zero(i) = strcmpi(char(F(i)), '0');
end

figure; image(F_zero, 'CDataMapping', 'scaled'); hold on; grid on;