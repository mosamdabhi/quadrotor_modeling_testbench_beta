function [phiDes, thetaDes, u1Des] = small_angle_position_controller(errorPosition, errorVelocity, commandHeading, commandAcc)
    global g m;
    
    kP = [3.0; 3.0; 15.0];
    kV = [4.5; 4.5; 9.0];
   
    % Simple PD control to compute desired linear acceleration in world 
    % frame
    linAccDes = commandAcc + kV.*errorVelocity + kP.*errorPosition + [0; 0; g];
    
    % Compute desired roll, pitch angles and desired control input in body
    % Z axis (u1Des)
    phiDes = (1/g)*(linAccDes(1)*sin(commandHeading(1)) - linAccDes(2)*cos(commandHeading(1)));
    thetaDes = (1/g)*(linAccDes(1)*cos(commandHeading(1)) - linAccDes(2)*sin(commandHeading(1)));
    u1Des = m*linAccDes(3);
end