function [U] = small_angle_attitude_controller(eEuler, eAngVel, u1Des)
    global I m g;
    kEuler = [I(1,1)*81; I(2,2)*81; I(3,3)*81];
    kAngVel = [2*I(1,1)*1*9; 2*I(2,2)*1*9; 2*I(1,1)*1*9];
    
    uTorques = kEuler.*eEuler + kAngVel.*eAngVel;
    
    %For attitude controller testing
    %u1Des = 0.2*m*g;
    
    U = [u1Des, uTorques'];
end