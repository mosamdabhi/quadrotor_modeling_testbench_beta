%%% Main script of controllers
function [U] = controller_unit_test()
addpath('../utilities');
addpath('../config');
addpath('../controllers');
hulk1c;
global cT cQ;
%% Defining initial params: % Making GUI based on this in next version
e_pos = [0;0;1]; e_vel = [0;0;0];
cmd_acc = [0;0;0]; mass = 1.2541;
control_state_angles = [0;0;0];
cmd_heading = [0;0;0];
cmd_jerk = [0;0;0];
ctrl_state_rollRate = 0; % Roll body angular rate (rad/s, x forward/y right/z down)
ctrl_state_pitchRate = 0; % Pitch body angular rate (rad/s, x forward/y right/z down)
ctrl_state_yawRate = 0; % Yaw body angular rate (rad/s, x forward/y right/z down)

%% Calling position controller
% Inputs:
% 1) e_pos = commanded_pos - local_pos;
% 2) e_vel = commanded_vel - local_vel;
% 3) cmd_acc = commanded acceleration
% 4) mass
% 5) control_state_angles -> control state euler angles
% 6) cmd_heading -> commanded heading. Should be a 3x1 column vector
% [cmd_heading(1); cmd_heading(2); cmd_heading(3)] -> in this way
% 7) cmd_jerk -> commanded jerk. Used as an input to function:
% computeAngularReference. It should be a 3x1 column vector
% [cmd_jerk(1); cmd_jerk(2); cmd_jerk(3)] -> in this way
%%%%%%%%%%%%%%%%
% Outputs:
% 1) att_cmd_eulerAngles
% 2) att_cmd_angVel 
% 3) att_cmd_angAcc 
% 4) att_cmd_thrust 
[att_cmd_eulerAngles, att_cmd_angVel, att_cmd_angAcc, att_cmd_thrust] = positionController(e_pos, e_vel, cmd_acc, mass, control_state_angles, cmd_heading, cmd_jerk);

%% Calling Inner loop
% Inputs:
% 1) ctrl_state_angles
% 2) att_cmd_eulerAngles -> coming from outer loop. 
% 3) ctrl_state_rollRate, ctrl_state_pitchRate, ctrl_state_yawRate
% 4) att_cmd_angVel -> Calculated and taken from outer loop
% 5) att_cmd_angAcc -> Calculated and taken from outer loop
% 6) att_cmd_thrust -> Calculated and taken from outer loop
%%%%%%%%%%%%
% Output:
% final_rpm motor values
[fin_rpm] = attitudeController(control_state_angles, att_cmd_eulerAngles, ctrl_state_rollRate, ctrl_state_pitchRate, ctrl_state_yawRate, att_cmd_angVel, att_cmd_angAcc, att_cmd_thrust);
U = motor_relationship_model(fin_rpm', cT, cQ);
end