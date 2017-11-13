clear all; %close all; clc;

str = {...'hover', 'up-down', 'up-down:_pirouette', ...
       ...'circle:_constant_heading'...
       ...'circle:_radial_heading', ...
       ...'circle:_pirouette', ...
       ...'spiral:_constant_heading', ...
       ...'spiral:_radial_heading', ...
       ...'spiral:_pirouette', ...
       'pirouette:_spin-in-place', ...
       ...'square:_constant_heading', ...
       ...'square:_turn_corners', ...
       ...'volume'
       };
   
for s = 1;%1:length(str)

    trajectory = str{s};
    start_waypoint = ones(4,1);
    circle_radius = .75;
    
    clear ROBOT_TRAJECTORY ROBOT_HISTORY ODOM 
    samples = 10000;
    ROBOT_HISTORY= struct();
    ROBOT_HISTORY.idx = 1;
    ROBOT_HISTORY.time = nan(1,samples);
    ROBOT_HISTORY.store_posDes = nan(4,samples);
    ROBOT_HISTORY.store_velDes = nan(4,samples);
    ROBOT_HISTORY.store_accDes = nan(4,samples);
    ROBOT_HISTORY.store_jrkDes = nan(4,samples);
    ROBOT_HISTORY.store_qdes = nan(4,samples);    

    [ROBOT_TRAJECTORY] = retrieve_trajectory(trajectory,start_waypoint,circle_radius);
    t_end = ROBOT_TRAJECTORY.timeline(end);
    
    for elapsed_t = 0:.1:t_end

        
        [pos_des,vel_des,acc_des,jrk_des] = ...
                set_desired( ROBOT_TRAJECTORY, elapsed_t, circle_radius ); 

        ROBOT_HISTORY.idx = ROBOT_HISTORY.idx+1;
        ROBOT_HISTORY.time(ROBOT_HISTORY.idx) = elapsed_t;
        ROBOT_HISTORY.store_posDes(:,ROBOT_HISTORY.idx) = pos_des;            
        ROBOT_HISTORY.store_velDes(:,ROBOT_HISTORY.idx) = vel_des;        
        ROBOT_HISTORY.store_accDes(:,ROBOT_HISTORY.idx) = acc_des;        
        ROBOT_HISTORY.store_jrkDes(:,ROBOT_HISTORY.idx) = jrk_des;        

    end

    fake = length(ROBOT_HISTORY.time);    
    ODOM.odom_time = nan(1,fake);
    ODOM.pos_odom = nan(3,fake);
    ODOM.vel_odom = nan(3,fake);    
    ODOM.att_odom = nan(3,fake);
    ODOM.ang_vel = nan(3,fake);
    
%     trimming_stuff = struct('desired_z_height',0,...
%                             'desired_start_hover', 0,...
%                             'record_of_gains',  0,...
%                             'roll_des_set', 0,...
%                             'roll_des', 0,...
%                             'num_rolls', 0,...
%                             'roll_time', 0,...
%                             'roll_recover', 0);  

    plot_selected({'position'},ROBOT_HISTORY,ROBOT_TRAJECTORY,[],ODOM,0,0,trajectory,[], [])

       h = axes();
       set(h,'Visible','off')
        behavior = trajectory;
        underscores = strfind(behavior,'_');
        temp = '';
        if ~isempty(underscores)
           for i = 1:length(underscores)
               if i==1
                   foo = 1;
               else
                   foo = underscores(i-1)    ;     
               end
               bar = underscores(i)-1;
               temp = strcat(temp,behavior(foo:bar));
               temp = strcat(temp,'\');
           end
           temp = strcat(temp,behavior(underscores(end):end));
        end
        if ~isempty(temp)
            behavior = temp;
        end
       title({behavior;' '},'fontsize',16)
       set(findall(gca, 'type', 'text'), 'visible', 'on')
       uistack(gca, 'bottom')
    drawnow
end