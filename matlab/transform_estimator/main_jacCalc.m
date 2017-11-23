function [FM, FI, JTheta] = main_jacCalc(values_sub)
%% Declaring syms and taking values in double
syms vB_B_x vB_B_y vB_B_z qIB_w qIB_x qIB_y qIB_z omegaX_B omegaY_B omegaZ_B b_a_x b_a_y b_a_z;
syms n1 n2 n3 n4 c_T c_m c_d jxx jyy jzz rBC_B_x rBC_B_y rBC_B_z omega_x omega_y omega_z; 
syms tau a_x a_y a_z w_a_x w_a_y w_a_z rBA_B_x rBA_B_y rBA_B_z;
syms FTotal_B_x FTotal_B_y FTotal_B_z MTotal_C_x MTotal_C_y MTotal_C_z;
syms rBC_B_x rBC_B_y rBC_B_z rCA_B_x rCA_B_y rCA_B_z w_F w_M rIB_I_x rIB_I_y rIB_I_z;

Net_subs_mat = [rIB_I_x; rIB_I_y; rIB_I_z; vB_B_x; vB_B_y; vB_B_z; qIB_x; qIB_y; qIB_z; qIB_w; ... 
                omegaX_B; omegaY_B; omegaZ_B;b_a_x; b_a_y; b_a_z; n1; n2; n3; n4; c_T; c_m; c_d; ...
                jxx; jyy; jzz; rBC_B_x; rBC_B_y; rBC_B_z; omega_x; omega_y; omega_z; tau; ...
                a_x; a_y; a_z; w_a_x; w_a_y; w_a_z; FTotal_B_x; FTotal_B_y; FTotal_B_z; MTotal_C_x; ...
                MTotal_C_y; MTotal_C_z; w_F; w_M];
            
% Based on eq(13-14) of conference paper, the values_sub vector needs to be generated
% once based on following pattern and they should match the ordering placed above:
% [rIB_I; vB_B; q_IB(x;y;z;w); omega_B; bias_in_acceleration; current_rpm(n1:n4); c_m; c_d; ...
%  c_J; rBC_B; omega_IMU; tau; acceleration_IMU; w_a (Acc IMU noise); ...
%  rBA_B; rCA_B; w_F; w_M (Normally Distributed Process Noise)];

%% Substitute values in the jacobians.
[FM, FI, JTheta] = jac_substitution(Net_subs_mat, values_sub);
end