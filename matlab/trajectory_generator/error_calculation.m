function [error] =  error_calculation(currentState, t, trajectory)
%% Defining initial parameters for cascaded controller
% For parametrized trajectory tracking, uncomment below line

  
[des_state_pos, des_state_vel, des_state_acc] = traj_param(t, trajectory);
des_state_velocity = des_state_vel;

%For step input uncomment the line below and enter desired step commands
%des_state_pos = [];
%des_state_velocity = [];

% If assuming perfect state estimation
errorPosition = des_state_pos' - currentState(7:9); 
errorVelocity = des_state_velocity' - currentState(1:3); 

% If assuming imperfect state estimation
% errorPosition = des_state_pos(1:3)' - currentState(20:22); 
% errorVelocity = des_state(4:6)' - currentState(17:19); 

error.p = errorPosition;
error.v = errorVelocity;

% Trajectory specified commanded heading and acceleration
error.heading = [0; 0; 0];
error.acc = [des_state_acc(1); des_state_acc(2); des_state_acc(3)];

% Outer Loop errors for plotting
outerLoopData = horzcat(t, errorPosition', errorVelocity', des_state_pos, des_state_velocity);

error.outerLoopData = outerLoopData;
end