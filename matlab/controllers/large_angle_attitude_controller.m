function [U] = large_angle_attitude_controller(attErr, u1Des, controller)
    eR = attErr.eR;
    eOmega = attErr.eOm;
    
    % Commanded torque computation
    U(1) = u1Des;
    
    % For inner loop tuning
    % global m g;
    % U(1) = m*g;
    
    U(2:4) = (-(controller.kR).*eR - (controller.kOm).*eOmega);
end

