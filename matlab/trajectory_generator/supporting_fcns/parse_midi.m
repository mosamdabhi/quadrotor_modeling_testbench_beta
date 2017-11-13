function [Kpq, Komega, Kp_pos, Kd_pos, trims] = parse_midi(keyboard_input, ...
                          Kpq_orig, Komega_orig, Kp_pos_orig, Kd_pos_orig, trims_orig)
            
% this function takes input from the midi keyboard & passes back new gains.
% if the keyborad_input is all zeros, it does nothing


if sum(sum(keyboard_input))==0
    Kpq = Kpq_orig;
    Komega = Komega_orig;
    Kp_pos = Kp_pos_orig;
    Kd_pos = Kd_pos_orig;
    trims = trims_orig;
else    
    kb = keyboard_input;
    
    kb_Kpq = eye(3);        kb_Kpq([1,5,9]) = kb(2,1:3); 
    kb_Komega = eye(3);     kb_Komega([1,5,9]) = kb(1,1:3); 
    kb_Kp_pos = eye(3);     kb_Kp_pos([1,5,9]) = kb(2,4:6); 
    kb_Kd_pos = eye(3);     kb_Kd_pos([1,5,9]) = kb(1,4:6); 

    D = 20;
    P = 200;

    Kpq = Kpq_orig + kb_Kpq*P;
    Komega = Komega_orig + kb_Komega*D;
    Kp_pos = Kp_pos_orig + kb_Kp_pos*P;
    Kd_pos = Kd_pos_orig + kb_Kd_pos*D;      
    
    % knobs are negative, sliders are positive
    trims = trims_orig;
    trim_max = 5*pi/180;
    trims.phi = trims_orig.phi + (-kb(1,7)+kb(2,7))*trim_max;
    trims.theta = trims_orig.theta + (-kb(1,8)+kb(2,8))*trim_max;
    trims.psi = trims_orig.psi + (-kb(1,9)+kb(2,9))*trim_max;
end


end                      
                      