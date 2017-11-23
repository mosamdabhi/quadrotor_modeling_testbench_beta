%% Clear workspace
clc; clear all; close all;

%% add path for Tim's matlab_utils
addpath('../../matlab_utils/src/');

%% Constants/syms/Varibales declaration
syms n1 n2 n3 n4;
syms c_T c_m c_d w_M w_F;
syms rBA_B_x rBA_B_y rBA_B_z rCA_B_x rCA_B_y rCA_B_z 
syms vB_B_x vB_B_y vB_B_z;
syms omegaX_B omegaY_B omegaZ_B;
syms rIB_I_x rIB_I_y rIB_I_z;
syms rBC_B_x rBC_B_y rBC_B_z;
syms qIB_w qIB_x qIB_y qIB_z; 
syms u1 u2 u3 u4 tau;
syms w_n_Noise_1 w_n_Noise_2 w_n_Noise_3 w_n_Noise_4;
syms jxx jyy jzz;
syms b_a_x b_a_y b_a_z b_omega_x b_omega_y b_omega_z;

syms FTotal_B_x FTotal_B_y FTotal_B_z MTotal_C_x MTotal_C_y MTotal_C_z; % New syms additions as these are automatically taken input from quad_dyn_main.m 

gravity_I = [0; 0; 9.80665];
mass = 1.251;

vB_B = [vB_B_x; vB_B_y; vB_B_z];
omega_B = [omegaX_B; omegaY_B; omegaZ_B];
r_BA_B = [rBA_B_x; rBA_B_y; rBA_B_z];
r_CA_B = [rCA_B_x; rCA_B_y; rCA_B_z];
rIB_I = [rIB_I_x; rIB_I_y; rIB_I_z];
rBC_B = [rBC_B_x; rBC_B_y; rBC_B_z];
qIB = [qIB_x; qIB_y; qIB_z; qIB_w];
b_omega_Total_dot = [0; 0; 0];
b_a_Total_dot = [0; 0; 0];
CBA = eye(3);

e_z = [0;0;1];
nTotal = [n1; n2; n3; n4];

T_init = [c_T*(n1^2); c_T*(n2^2); c_T*(n3^2); c_T*(n4^2)];                  % Eq(3)
T_init_vect = [T_init(1)*e_z, T_init(2)*e_z, T_init(3)*e_z, T_init(4)*e_z]; % Eq(3)

M_init_vect = [(c_m*(n1^2) + w_M)*e_z, (c_m*(n2^2) + w_M)*e_z, ...  
               (c_m*(n3^2) + w_M)*e_z, (c_m*(n4^2) + w_M)*e_z];             % Eq(4)

v_Hub = transpose(CBA) * (vB_B + cross(omega_B,r_BA_B));                    % Eq(6) 

F_init = [CBA * (T_init_vect(:,1) - T_init(1)*[c_d 0 0; 0 c_d 0; 0 0 0]*v_Hub + w_F), ...
          CBA * (T_init_vect(:,2) - T_init(2)*[c_d 0 0; 0 c_d 0; 0 0 0]*v_Hub + w_F), ...
          CBA * (T_init_vect(:,3) - T_init(3)*[c_d 0 0; 0 c_d 0; 0 0 0]*v_Hub + w_F), ...
          CBA * (T_init_vect(:,4) - T_init(4)*[c_d 0 0; 0 c_d 0; 0 0 0]*v_Hub + w_F)];  % Eq(5)

%FTotal_B = F_init(:,1) + F_init(:,2) + F_init(:,3) + F_init(:,4); % Eq(7)
FTotal_B = [FTotal_B_x; FTotal_B_y; FTotal_B_z];


M_Tot = [CBA*M_init_vect(:,1) + cross(F_init(:,1),r_CA_B), ...
         CBA*M_init_vect(:,2) + cross(F_init(:,2),r_CA_B), ...
         CBA*M_init_vect(:,3) + cross(F_init(:,3),r_CA_B), ...
         CBA*M_init_vect(:,4) + cross(F_init(:,4),r_CA_B)];       % Eq(8)

%MTotal_C = M_Tot(:,1) + M_Tot(:,2) + M_Tot(:,3) + M_Tot(:,4);     % Eq(8)
MTotal_C = [MTotal_C_x; MTotal_C_y; MTotal_C_z];

prompt = 'Value of qIB_w : ';
act_val_qIB_w = input(prompt);
min_q_temp = [qIB_x; qIB_y; qIB_z];
new_qIB = subs(sign(qIB_w)*min_q_temp,qIB_w,sign(act_val_qIB_w));

b_a_Total = [b_a_x; b_a_y; b_a_z];
b_omega_Total = [b_omega_x; b_omega_y; b_omega_z];

uTotal = [u1; u2; u3; u4];
w_n_NoiseTotal = [w_n_Noise_1; w_n_Noise_2; w_n_Noise_3; w_n_Noise_4];
j_C = [jxx; jyy; jzz];
J_C = diag(j_C);
omegaSkew = [0 -omegaZ_B omegaY_B; omegaZ_B 0 -omegaX_B; -omegaY_B omegaX_B 0];
ohmOmega = [-omegaSkew omega_B; -transpose(omega_B) 0]/2;

CIB = [  1 - 2*qIB_y^2 - 2*qIB_z^2, 2*(qIB_x*qIB_y - qIB_z*qIB_w), 2*(qIB_x*qIB_z + qIB_y*qIB_w);
       2*(qIB_x*qIB_y + qIB_z*qIB_w),   1 - 2*qIB_x^2 - 2*qIB_z^2, 2*(qIB_y*qIB_z - qIB_x*qIB_w);
       2*(qIB_x*qIB_z - qIB_y*qIB_w), 2*(qIB_y*qIB_z + qIB_x*qIB_w),   1 - 2*qIB_x^2 - 2*qIB_y^2];

temp_omegaBDot_calc = J_C*omega_B;
omega_B_dot = inv(J_C)*(MTotal_C - cross(omega_B, temp_omegaBDot_calc));    % Eq(15(d))
   
Temp_vBDot_1 = CIB*(FTotal_B/mass);
Temp_vBDot_2 = gravity_I;
Temp_vBDot_3 = cross(omega_B, vB_B);
Temp_vBDot_4 = cross(omega_B_dot, rBC_B);
Temp_vBDot_5_1 = cross(omega_B, rBC_B);
Temp_vBDot_5_2 = cross(omega_B, Temp_vBDot_5_1);


%% f function definition
rIB_I_dot = vB_B;                                                           % Eq(15(a))
vB_B_dot = Temp_vBDot_1 - Temp_vBDot_2 - Temp_vBDot_4 - Temp_vBDot_5_2;     % Eq(15(b))
qIB_dot = [-omegaSkew/2 omega_B/2; -transpose(omega_B/2) 0]*qIB;            % Eq(15(c))
nTotal_dot = (uTotal - nTotal)/tau; %+ w_n_NoiseTotal;                      % Eq(9) & Eq(15(e))

%% x_vars calculation
x_vars = [c_T; c_m; c_d; j_C; rBC_B];
%minimal_x_vars = [rIB_I; vB_B; new_qIB; omega_B; b_omega_Total; b_a_Total; nTotal];

%% Jacobian Calculation
f = simplify([rIB_I_dot; vB_B_dot; qIB_dot; omega_B_dot; b_omega_Total_dot; b_a_Total_dot; nTotal_dot], 'Steps', 50);
F = simplify(derive_analytic_jacobian(f, x_vars), 'Steps', 50);

%% Plotting functionality
F_zero = zeros(size(F));
for i = 1:numel(F_zero)
    F_zero(i) = strcmpi(char(F(i)), '0');
end

save ('Jacobian_MatFiles/Jac_jTheta_sym.mat', 'F');
figure; image(F_zero, 'CDataMapping', 'scaled'); hold on; grid on;