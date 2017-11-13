function [attitudeErr] = attitude_error_calculation(currentState, attCommands, currentQuat, t)
%% Inner loop controller tuning
% desEulerAng = [0*pi/180, 0*pi/180, 0*pi/180];
% attCommands.commandQuat = ZYXToQuat(desEulerAng);
% attCommands.commandAngVel = [0, 0, 0];

%% Attitude commands
commandQuat = attCommands.commandQuat;
commandAngVel = attCommands.commandAngVel;

%% Current angular rates
angularRates = currentState(4:6); % Body frame angular rates

%% Change the desired Euler Angles to tune/test attitude controller separately
QuatforEuler = [commandQuat.w, commandQuat.x, commandQuat.y, commandQuat.z];

temp = quat2eul(QuatforEuler);
desEulerAng(1) = temp(3);
desEulerAng(2) = temp(2);
desEulerAng(3) = temp(1);

%% Error in orientation and error in angular velocity orientation
% Current direction cosine matrix
R = QuatToR(currentQuat);

% Desired direction cosine matrix
Rdes = QuatToR(commandQuat); 

% Error in orientation
%eR = 0.5*(vee(Rdes'*R - R'*Rdes)); % if using exisiting error metric
eR = (vee(Rdes'*R - R'*Rdes))/(2*sqrt(1+trace(Rdes'*R))); % if using rectified error metric based on Taeyoung Lee's paper

% Error in angular velocity
eOmega = angularRates - R'*Rdes*commandAngVel';

% Inner Loop errors for plotting
innerLoopData = horzcat(t, eR', eOmega', desEulerAng, commandAngVel);

%% Return structure attitudeErr
attitudeErr.eR = eR;
attitudeErr.eOm = eOmega;
attitudeErr.innerLoopData = innerLoopData;

end