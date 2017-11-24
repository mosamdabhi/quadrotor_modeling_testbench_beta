function [fin_ctrl] = epc_init(desPos, desVel, desAcc)
% desPos = [1;1;1]; desVel = [1;1;1]; desAcc = [-1;0;0];
HORIZON_STATES = 1;
A = zeros(6,6);
A(1,4) = 1; A(2,5) = 1; A(3,6) = 1;
mass = 1.251;
B(4,1) = 1/mass; B(5,2) = 1/mass; B(6,3) = 1/mass;
C = zeros(3,6); D = zeros(3,3);
T = 0.001;
ans = expm([A*T B*T; C*T D*T]);
A_discrete = ans(1:6,1:6);
B_discrete = ans(1:6,7:9);
cal_B = B_discrete;
cal_B_transpose = transpose(cal_B);
% cal_r calculation
cal_r = repmat([desPos; desVel], HORIZON_STATES, 1);
cal_c = zeros(HORIZON_STATES*6,1);

% Q Calculation for dynamic N
Q_tmp = [199.464053 199.464053 400.693315 30.07 53.85 51.757]; %N=2 Hacked values working
Q = diag(Q_tmp);
Q_cell = repmat({Q}, 1, HORIZON_STATES);
cal_Q = blkdiag(Q_cell{:});

% R Calculation for dynamic N
R_tmp = [0.002660 0.002660 0.001564]; %N=2 hacked working

R = diag(R_tmp);
R_cell = repmat({R}, 1, HORIZON_STATES);
cal_R = blkdiag(R_cell{:});

% cal_H
cal_H = ((cal_B_transpose*cal_Q)*cal_B) + cal_R;
inverse_cal_H = inv(cal_H);

% cal_h
temp_for_cal_h = (cal_c - cal_r);
cal_h = (cal_B_transpose*cal_Q)*temp_for_cal_h;

cmd_acc = repmat([desAcc], HORIZON_STATES, 1);
% cmd_acc = [-1;0;0];
% cmd_acc = [0.6663; -0.7457; 0];
% u = (inverse_cal_H*(-cal_h));
% ctrl(1) = u(1) + cmd_acc(1)*mass;
% ctrl(2) = u(2) + cmd_acc(2)*mass;
% ctrl(3) = -(u(3) + (cmd_acc(3) - 9.80665)*mass);
fin_ctrl = mass*(inverse_cal_H*(-cal_h) + [0; 0; 9.80665] + cmd_acc);
fin_ctrl = fin_ctrl(1:3);
end



