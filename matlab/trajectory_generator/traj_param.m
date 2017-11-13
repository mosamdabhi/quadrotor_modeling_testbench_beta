function [des_state_pos, des_state_vel, des_state_acc] = traj_param(t, sel_traj)
%Trajectory generatiion using parameterized equations
% Takeoff to initial leminiscate position (x_0,y_0,1) and track

switch sel_traj
    case 'Circle'
        % position
        des_state_pos(1) = cos(t); %(cos(t))./(1+(sin(t)*sin(t)));
        des_state_pos(2) = sin(t); 
        des_state_pos(3) = 1;
        
        % velocity
        des_state_vel(1) = -sin(t);
        des_state_vel(2) = cos(t);
        des_state_vel(3) = 0;
    
        % acceleration (feedforward term)
        des_state_acc(1) = -cos(t);
        des_state_acc(2) = -sin(t);
        des_state_acc(3) = 0;

    case 'Leminiscate'        
        % position
        des_state_pos(1) = (cos(t))./(1+(sin(t)*sin(t)));
        des_state_pos(2) = (sin(t).*cos(t))./(1+(sin(t).*sin(t)));
        des_state_pos(3) = 1;

        % velocity
        des_state_vel(1) = - sin(t)/(sin(t)^2 + 1) - (2*cos(t)^2*sin(t))/(sin(t)^2 + 1)^2;
        des_state_vel(2) = cos(t)^2/(sin(t)^2 + 1) - sin(t)^2/(sin(t)^2 + 1) - (2*cos(t)^2*sin(t)^2)/(sin(t)^2 + 1)^2;
        des_state_vel(3) = 0;

        % acceleration (feedforward term)
        des_state_acc(1) = (6*cos(t)*sin(t)^2)/(sin(t)^2 + 1)^2 - (2*cos(t)^3)/(sin(t)^2 + 1)^2 - cos(t)/(sin(t)^2 + 1) + (8*cos(t)^3*sin(t)^2)/(sin(t)^2 + 1)^3;
        des_state_acc(2) = (6*cos(t)*sin(t)^3)/(sin(t)^2 + 1)^2 - (4*cos(t)*sin(t))/(sin(t)^2 + 1) - (6*cos(t)^3*sin(t))/(sin(t)^2 + 1)^2 + (8*cos(t)^3*sin(t)^3)/(sin(t)^2 + 1)^3;
        des_state_acc(3) = 0;

        case 'Eight Curve'
        % position
        des_state_pos(1) = sin(t);
        des_state_pos(2) = sin(t)*cos(t);
        des_state_pos(3) = 1;

        % velocity
        des_state_vel(1) = cos(t);
        des_state_vel(2) = cos(t)^2 - sin(t)^2;
        des_state_vel(3) = 0;

        % acceleration (feedforward term)
        des_state_acc(1) = -sin(t);
        des_state_acc(2) = -4*cos(t)*sin(t);
        des_state_acc(3) = 0;
            
        case 'Ellipse'
        % position
        des_state_pos(1) = -(sin(t)/(sqrt(cos(t)^2 + sin(t)^2)));
        des_state_pos(2) = (cos(t)/(sqrt(cos(t)^2 + sin(t)^2)));
        des_state_pos(3) = 1;

        % velocity
        des_state_vel(1) = -cos(t)/(cos(t)^2 + sin(t)^2)^(1/2);
        des_state_vel(2) = -sin(t)/(cos(t)^2 + sin(t)^2)^(1/2);
        des_state_vel(3) = 0;

        % acceleration (feedforward term)
        des_state_acc(1) = sin(t)/(cos(t)^2 + sin(t)^2)^(1/2);
        des_state_acc(2) = -cos(t)/(cos(t)^2 + sin(t)^2)^(1/2);
        des_state_acc(3) = 0;

        case 'Cornoid'
        % position
        des_state_pos(1) = cos(t)*(1 - 2*(sin(t)^2));
        des_state_pos(2) = sin(t)*(1 + 2*(cos(t)^2));
        des_state_pos(3) = 1;

        % velocity
        des_state_vel(1) = sin(t)*(2*sin(t)^2 - 1) - 4*cos(t)^2*sin(t);
        des_state_vel(2) = cos(t)*(2*cos(t)^2 + 1) - 4*cos(t)*sin(t)^2;
        des_state_vel(3) = 0;

        % acceleration (feedforward term)
        des_state_acc(1) = cos(t)*(2*sin(t)^2 - 1) - 4*cos(t)^3 + 12*cos(t)*sin(t)^2;
        des_state_acc(2) = 4*sin(t)^3 - sin(t)*(2*cos(t)^2 + 1) - 12*cos(t)^2*sin(t);
        des_state_acc(3) = 0;

        case 'Astroid'
        % position
        des_state_pos(1) = 3*cos(t) + cos(3*t);
        des_state_pos(2) = 3*sin(t) - sin(3*t);
        des_state_pos(3) = 1; %(t^3)/100;

        % velocity
        des_state_vel(1) = - 3*sin(3*t) - 3*sin(t);
        des_state_vel(2) = 3*cos(t) - 3*cos(3*t);
        des_state_vel(3) = 0; %(3*t^2)/100;

        % acceleration (feedforward term)
        des_state_acc(1) = - 9*cos(3*t) - 3*cos(t);
        des_state_acc(2) = 9*sin(3*t) - 3*sin(t);
        des_state_acc(3) = 0; %(3*t)/50;  

    case 'Vertical Circle'
        % position
        des_state_pos(1) = cos(t); 
        des_state_pos(2) = 0; 
        des_state_pos(3) = sin(t)+3;
        
        % velocity
        des_state_vel(1) = -sin(t);
        des_state_vel(2) = 0;
        des_state_vel(3) = cos(t);
    
        % acceleration (feedforward term)
        des_state_acc(1) = -cos(t);
        des_state_acc(2) = 0;
        des_state_acc(3) = -sin(t);        
        
    case 'Bicorn'
        % position
        des_state_pos(1) = sin(t); 
        des_state_pos(2) = (cos(t)^2 * (2+cos(t)))/(3 + sin(t)^2); 
        des_state_pos(3) = 1;
        
        % velocity
        des_state_vel(1) = cos(t);
        des_state_vel(2) = - (cos(t)^2*sin(t))/(sin(t)^2 + 3) - (2*cos(t)*sin(t)*(cos(t) + 2))/(sin(t)^2 + 3) - (2*cos(t)^3*sin(t)*(cos(t) + 2))/(sin(t)^2 + 3)^2;
        des_state_vel(3) = 0;
    
        % acceleration (feedforward term)
        des_state_acc(1) = -sin(t);
        des_state_acc(2) = (4*cos(t)*sin(t)^2)/(sin(t)^2 + 3) - cos(t)^3/(sin(t)^2 + 3) - (2*cos(t)^2*(cos(t) + 2))/(sin(t)^2 + 3) - (2*cos(t)^4*(cos(t) + 2))/(sin(t)^2 + 3)^2 + (2*sin(t)^2*(cos(t) + 2))/(sin(t)^2 + 3) + (4*cos(t)^3*sin(t)^2)/(sin(t)^2 + 3)^2 + (10*cos(t)^2*sin(t)^2*(cos(t) + 2))/(sin(t)^2 + 3)^2 + (8*cos(t)^4*sin(t)^2*(cos(t) + 2))/(sin(t)^2 + 3)^3;
        des_state_acc(3) = 0;   

    case 'Butterfly Curve'
        % position
        des_state_pos(1) = sin(t)*(exp(cos(t)) - 2*cos(4*t) + (sin(t/12)^5)); 
        des_state_pos(2) = cos(t)*(exp(cos(t)) - 2*cos(4*t) + (sin(t/12)^5));
        des_state_pos(3) = 1;
        
        % velocity
        des_state_vel(1) = sin(t)*(8*sin(4*t) + (5*cos(t/12)*sin(t/12)^4)/12 - exp(cos(t))*sin(t)) + cos(t)*(exp(cos(t)) - 2*cos(4*t) + sin(t/12)^5);
        des_state_vel(2) = cos(t)*(8*sin(4*t) + (5*cos(t/12)*sin(t/12)^4)/12 - exp(cos(t))*sin(t)) - sin(t)*(exp(cos(t)) - 2*cos(4*t) + sin(t/12)^5);
        des_state_vel(3) = 0;
    
        % acceleration (feedforward term)
        des_state_acc(1) = sin(t)*(32*cos(4*t) - exp(cos(t))*cos(t) + (5*cos(t/12)^2*sin(t/12)^3)/36 - (5*sin(t/12)^5)/144 + exp(cos(t))*sin(t)^2) - sin(t)*(exp(cos(t)) - 2*cos(4*t) + sin(t/12)^5) + 2*cos(t)*(8*sin(4*t) + (5*cos(t/12)*sin(t/12)^4)/12 - exp(cos(t))*sin(t));
        des_state_acc(2) = cos(t)*(32*cos(4*t) - exp(cos(t))*cos(t) + (5*cos(t/12)^2*sin(t/12)^3)/36 - (5*sin(t/12)^5)/144 + exp(cos(t))*sin(t)^2) - cos(t)*(exp(cos(t)) - 2*cos(4*t) + sin(t/12)^5) - 2*sin(t)*(8*sin(4*t) + (5*cos(t/12)*sin(t/12)^4)/12 - exp(cos(t))*sin(t));
        des_state_acc(3) = 0;         

    case 'TakeOff'
        % position
        des_state_pos(1) = 1; %(cos(t))./(1+(sin(t)*sin(t)));
        des_state_pos(2) = 0; 
        des_state_pos(3) = 2;
        
        % velocity
        des_state_vel(1) = 0;
        des_state_vel(2) = 0;
        des_state_vel(3) = 0;
    
        % acceleration (feedforward term)
        des_state_acc(1) = 0;
        des_state_acc(2) = 0;
        des_state_acc(3) = 0;        

    case 'Spiral Circle'
        % position
        des_state_pos(1) = cos(t); 
        des_state_pos(2) = sin(t); 
        des_state_pos(3) = t/15;
        
        % velocity
        des_state_vel(1) = -sin(t);
        des_state_vel(2) = cos(t);
        des_state_vel(3) = 0;
    
        % acceleration (feedforward term)
        des_state_acc(1) = -cos(t);
        des_state_acc(2) = -sin(t);
        des_state_acc(3) = 0;

    case 'Circle_MultiDimensional'
        % position
        if t <= 6.2832
            des_state_pos(1) = cos(t); %(cos(t))./(1+(sin(t)*sin(t)));
            des_state_pos(2) = sin(t); 
            des_state_pos(3) = 3;

            % velocity
            des_state_vel(1) = -sin(t);
            des_state_vel(2) = cos(t);
            des_state_vel(3) = 0;

            % acceleration (feedforward term)
            des_state_acc(1) = -cos(t);
            des_state_acc(2) = -sin(t);
            des_state_acc(3) = 0;        
            
        elseif t > 6.2832 && t <= 12.5664
            des_state_pos(1) = 1; 
            des_state_pos(2) = sin(t); 
            des_state_pos(3) = -cos(t)+4;

            % velocity
            des_state_vel(1) = 0;
            des_state_vel(2) = cos(t);
            des_state_vel(3) = sin(t);

            % acceleration (feedforward term)
            des_state_acc(1) = 0;
            des_state_acc(2) = -sin(t);
            des_state_acc(3) = cos(t);  

        else
            des_state_pos(1) = cos(t); 
            des_state_pos(2) = 0; 
            des_state_pos(3) = sin(t)+3;

            % velocity
            des_state_vel(1) = -sin(t);
            des_state_vel(2) = 0;
            des_state_vel(3) = cos(t);

            % acceleration (feedforward term)
            des_state_acc(1) = -cos(t);
            des_state_acc(2) = 0;
            des_state_acc(3) = -sin(t);                    
        end

        
    otherwise
            display('Given trajectory not identified');


end

