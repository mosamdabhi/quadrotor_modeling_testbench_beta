%%% Main function for cascaded controller
function [constrainedRPM, innerLoopData, outerLoopData] = small_angle_control_main(currentState, t)
addpath('utilities');
addpath('config');
hulk1c;
% initial_config;
global des_state; 

%% Defining initial parameters for cascaded controller
%eulerOffset = [5*pi/180, 5*pi/180, 5*pi/180];

% For ramp input uncomment the line below
%[des_state_pos, des_state_euler] = ramp_input(currentState, des_state(1:3), des_state(7:9), eulerOffset, slope, t + 0.01);

%For step input uncomment the line below
des_state_pos = des_state;

% If assuming perfect state estimation
errorPosition = des_state_pos(1:3)' - currentState(7:9); errorVelocity = des_state(4:6)' - currentState(1:3); 
angularRates = currentState(4:6); % Body frame angular rates
eulerAngles = currentState(10:12); % [roll; pitch; yaw] (same as [phi; theta; psi])

% If assuming imperfect state estimation
errorPosition = des_state_pos(1:3)' - currentState(20:22); errorVelocity = des_state(4:6)' - currentState(17:19); 
angularRates = currentState(4:6); % Body frame angular rates
eulerAngles = currentState(10:12); % [roll; pitch; yaw] (same as [phi; theta; psi])

% Trajectory specified commanded heading and acceleration
commandHeading = [0; 0; 0];
commandAcceleration = [0; 0; 0];

%% Calling position controller
[phiDes, thetaDes, u1Des] = small_angle_position_controller(errorPosition, errorVelocity, commandHeading, commandAcceleration);

%% Change the desired Euler Angles to tune/test attitude controller separately
%desEulerAng = [des_state_euler(1); des_state_euler(2); des_state_euler(3)]; % [roll; pitch; yaw] (same as [phi; theta; psi])

%% Error in euler angles and body angular rates
desEulerAng = [phiDes; thetaDes; commandHeading(1)]; % [roll; pitch; yaw] (same as [phi; theta; psi])
errorEulerAng = (desEulerAng - eulerAngles); 

desAngVel = [0; 0; 0]; % Desired body rates kept zero for now
errorAngVel = desAngVel - angularRates;

%% Calling inner loop attitude controller
[uNoConstraint] = small_angle_attitude_controller(errorEulerAng, errorAngVel, u1Des);

%% RPM calculation
[constrainedRPM] = convert_bodyForces_toRPM(uNoConstraint);

%% Data for post processing
innerLoopData = horzcat(t, errorEulerAng', errorAngVel', desEulerAng', desAngVel');
outerLoopData = horzcat(t, phiDes', thetaDes', u1Des');
end