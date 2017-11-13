addpath('config')
global des_state
%% Reference commands for inner loop
prompt={'Enter a desired value of \phi (in degrees)', 'Enter a desired value of \theta (in degrees)', 'Enter a desired value of \psi (in degrees)', 'Enter a desired value of \omega_{x} (in degrees)', 'Enter a desired value of \omega_{y} (in degrees)', 'Enter a desired value of \omega_{z} (in degrees)'};
name = 'Inner loop Control Values';
defaultans = {'30','30','30','30','30','30'};
options.Interpreter = 'tex';
answer = inputdlg(prompt,name,[1 40],defaultans,options);
%%%% Converting final values in radians
d_phi = degtorad(str2num(answer{1}));
d_theta = degtorad(str2num(answer{2}));
d_psi = degtorad(str2num(answer{3}));
omega_des(1) = degtorad(str2num(answer{4}));
omega_des(2) = degtorad(str2num(answer{5}));
omega_des(3) = degtorad(str2num(answer{6}));

%% Call your function here for inner loop

%% Reference commands for outer loop
prompt={'Enter the desired position in x', 'Enter the desired position in y', 'Enter the desired position in z', 'Enter the desired velocity in x', 'Enter the desired velocity in y', 'Enter the desired velocity in z', 'Enter commanded acceleration in x', 'Enter commanded acceleration in y', 'Enter commanded acceleration in z'};
name = 'Outer loop Control Values';
defaultans = {'30','30','30','30','30','30','30','30','30'};
options.Interpreter = 'tex';
answer = inputdlg(prompt,name,[1 40],defaultans,options);
des_pos(1) = str2num(answer{1});
des_pos(2) = str2num(answer{2});
des_pos(3) = str2num(answer{3});
des_vel(1) = str2num(answer{4});
des_vel(2) = str2num(answer{5});
des_vel(3) = str2num(answer{6});
cmd_acc(1) = str2num(answer{7});
cmd_acc(2) = str2num(answer{8});
cmd_acc(3) = str2num(answer{9});
 
%% Call outer loop controller fucntion