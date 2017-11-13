function X = init_state(sel_traj)
%% Initialization
    X = zeros(12,1);
    %% Defining initial position based on trajectory
switch sel_traj
    case 'Circle'
        X(7) = 1;
        X(8) = 0;
        X(9) = 1;    
    case 'Leminiscate'        
        X(7) = 1;
        X(8) = 0;
        X(9) = 1;
    case 'Eight Curve'
        X(7) = 0;
        X(8) = 0;
        X(9) = 1;  
    case 'Ellipse'
        X(7) = 0;
        X(8) = 1;
        X(9) = 1;          
    case 'Cornoid'
        X(7) = 1;
        X(8) = 0;
        X(9) = 1;                  
    case 'Astroid'
        X(7) = 4;
        X(8) = 0;
        X(9) = 1;   
    case 'Vertical Circle'
        X(7) = 1;
        X(8) = 0;
        X(9) = 3;  
    case 'Bicorn'
        X(7) = 0;
        X(8) = 1;
        X(9) = 1;
    case 'Butterfly Curve'
        X(7) = 0;
        X(8) = 0.7183;
        X(9) = 1;
    case 'TakeOff'
        X(7) = 0;
        X(8) = 0;
        X(9) = 2;         
    case 'Spiral Circle'
        X(7) = 1;
        X(8) = 0;
        X(9) = 0.2;   
    case 'Circle_MultiDimensional'
        X(7) = 1;
        X(8) = 0;
        X(9) = 3;           
    otherwise
        display('Invalid initial state');
 
    %% Defining initial euler angles
    X(10) = 0*pi/180;
    X(11) = 0*pi/180;
    X(12) = 0*pi/180;    
end