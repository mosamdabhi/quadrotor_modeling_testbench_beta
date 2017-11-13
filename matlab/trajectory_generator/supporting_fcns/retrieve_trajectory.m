function [ROBOT_TRAJECTORY] = retrieve_trajectory(name,start_waypoint,circle_radius)

%% some constants between trajectories, to make life easier
num_reps = 2;

hover_time = 1;
rise_time = 4;
land_time = 4;
out_time = 3;
return_time = out_time; %3;
line_length = 2;

circle_speed = .45; %[m/s] .45   %.75->xy+/-1, 1->1.3, 

pirouette_time = 6; % [s]

num_spirals = 2;
spiral_height = 1;
hh = .75; % hover height          

%% ...and the trajectories themselves

switch name
    case 'take_off'
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time ]); 
        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},...
                                      {'hovering'}];
        ROBOT_TRAJECTORY.waypoints = repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), ...
              [0;0;hh;0] ]; 
        ROBOT_TRAJECTORY.state = ''; 
        
    case 'land'
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 land_time ]); 
        ROBOT_TRAJECTORY.behavior = [ {'landing'},...                              
                                      {'landed.'} ];
        ROBOT_TRAJECTORY.waypoints = repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ [0;0;hh;0], ...      
                [zeros(3,1);0] ];
        ROBOT_TRAJECTORY.state = ''; 
            
    case 'hover'
        % time per behavior, including time to travel from start -> hover-start ->
        % behavior start, and behavior-end > hover-end -> land
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time hover_time land_time ]); 
        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},...
                                      {'hovering'},...
                                      {'landing'},...                              
                                      {'landed.'} ];
        ROBOT_TRAJECTORY.waypoints = repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), ... 
                [0;0;hh;0], ... 
                [0;0;hh;0], ...         
                [zeros(3,1);0] ]; 
        ROBOT_TRAJECTORY.state = ''; 

    case 'line_x'
        % time per behavior, including time to travel from start -> hover-start ->
        % behavior start, and behavior-end > hover-end -> land
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time hover_time ...
                                             out_time,out_time,... 
                                             hover_time land_time ]); 
        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},...
                                      {'hovering'},...
                                      {'out'},{'back'},...
                                      {'hovering'},{'landing'},...                              
                                      {'landed.'} ];
        ROBOT_TRAJECTORY.waypoints = repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), ... start
                [0;0;hh;0], ... go up
                [0;0;hh;0], ... hover
                [line_length;0;hh;0],... out
                [0;0;hh;0],... back
                [0;0;hh;0], ... hover            
                [zeros(3,1);0] ]; % land
        ROBOT_TRAJECTORY.state = ''; 

    case 'line_y'
        % time per behavior, including time to travel from start -> hover-start ->
        % behavior start, and behavior-end > hover-end -> land
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time hover_time ...
                                             out_time,out_time,... 
                                             hover_time land_time ]); 
        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},...
                                      {'hovering'},...
                                      {'out'},{'back'},...
                                      {'hovering'},{'landing'},...                              
                                      {'landed.'} ];
        ROBOT_TRAJECTORY.waypoints = repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), ... start
                [0;0;hh;0], ... go up
                [0;0;hh;0], ... hover
                [0;line_length;hh;0],... out
                [0;0;hh;0],... back
                [0;0;hh;0], ... hover            
                [zeros(3,1);0] ]; % land
        ROBOT_TRAJECTORY.state = ''; 
        
    case 'up-down'
        % time per behavior, including time to travel from start -> hover-start ->
        % behavior start, and behavior-end > hover-end -> land
        updown = zeros(4,num_reps*2);
        updown(3,:) = hh + repmat(.5*[1 -1],1,num_reps);
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time hover_time ...
                                             repmat([2 2], 1, num_reps),... % updown
                                             2 hover_time land_time ]); % recover hover land
        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},...
                                      {'hovering'},...
                                      repmat([{'up'},{'down'}],1,num_reps),...
                                      {'recover'},{'hovering'},{'landing'},...                              
                                      {'landed.'} ];
        ROBOT_TRAJECTORY.waypoints = repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), ... start
                [0;0;hh;0], ... go up
                [0;0;hh;0], ... hover
                updown,...
                [0;0;hh;0], [0;0;hh;0], ... recover hover            
                [zeros(3,1);0] ]; % land
        ROBOT_TRAJECTORY.state = ''; 

    case 'up-down:_pirouette'
%%
        % time per behavior, including time to travel from start -> hover-start ->
        % behavior start, and behavior-end > hover-end -> land
        updown = zeros(4,num_reps*2);
        updown(3,:) = hh + repmat(.5*[1 -1],1,num_reps);
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time hover_time ...
                                             5 pirouette_time repmat([pirouette_time pirouette_time], 1, num_reps-1),... % updown
                                             5 hover_time land_time ]); % recover hover land
        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},...
                                      {'hovering'},...
                                      {'go-to'},{'spin-down'}, repmat([{'spin-up'},{'spin-down'}],1,num_reps-1),...
                                      {'recover'},{'hovering'},{'landing'},...                              
                                      {'landed.'} ];
                                  
        % initialize with zeros
        ROBOT_TRAJECTORY.waypoints(:,:,1:length(ROBOT_TRAJECTORY.behavior)) = ...
                                    zeros(4,3,length(ROBOT_TRAJECTORY.behavior));
        % stick in positions
        ROBOT_TRAJECTORY.waypoints(:,1,:) = repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), [0;0;hh;0], ... start, go up
                [0;0;hh;0], ... hover
                updown, ...
                [0;0;hh;0], [0;0;hh;0], ... recover hover            
                [zeros(3,1);0] ]; % land
            
        % setting yaw to repmat -6*pi means we want to yaw 3 times per
        % large circle       
        yaw_vec = (1:num_reps*2)*-2*pi+2*pi;
        ROBOT_TRAJECTORY.waypoints(4,1,4:end) = [yaw_vec repmat(yaw_vec(end),1,3)];         
        
        % velocity at start of pirouettes
        [~,vpir,apir,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(4)+.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,4) = [vpir,apir];

        % velocity at end of circles
        [~,vpir,apir,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(end-3)-.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,end-3) = [vpir,apir];        
            
        ROBOT_TRAJECTORY.state = ''; 
            
    case 'circle:_constant_heading'
        circumference = 2*pi*circle_radius^2;
        circle_speeds = circle_speed*ones(1,num_reps);
        times = circumference./circle_speeds;
        circle_times = zeros(1,2*length(circle_speeds));
        circle_times(2:2:end) = times;
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time hover_time ... % up + hover
                                             out_time ... % to start
                                             circle_times...
                                             return_time  land_time ]); % land
        num_static = 6; % 6 = start, up, hover, to-circle-start, back, land
        num_circles = (length(ROBOT_TRAJECTORY.timeline)-num_static)/2;  

        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},{'hovering'}, {'to_start'},...
              repmat([{'circle'},{'circle_constant_heading'}],1,num_circles), ...                                                  
              {'hover_end'}, {'landing'}, {'landed.'} ];

        ROBOT_TRAJECTORY.waypoints(:,:,1:length(ROBOT_TRAJECTORY.behavior)) = ...
                                    zeros(4,3,length(ROBOT_TRAJECTORY.behavior));

        ROBOT_TRAJECTORY.waypoints(:,1,:) = ...
            repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), [0;0;hh;0], [0;0;hh;0], ... 0+2+5
                [0;circle_radius;hh;0], ... % to start of circle
                repmat([ [0;0;hh;0] ,... % 0 just to set center of circle
                         [0;circle_radius;hh;-2*pi] ], ... % 15: CIRCLE 
                         1,num_circles) ...         
                [0;0;hh;num_circles*-2*pi], ... % landing       
                [zeros(3,1);num_circles*-2*pi] ]; % land
        yaw_vec = reshape(repmat((1:num_circles)*-2*pi,2,1),1,(num_circles)*2);
        ROBOT_TRAJECTORY.waypoints(4,1,num_static+1:end-2) = yaw_vec(2:end-1);
        
        % constant yaw
        ROBOT_TRAJECTORY.waypoints(4,1,:) = 0;
        
        % velocity at start of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(4)+.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,4) = [vcircle,acircle];

        % velocity at end of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(end-2)-.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,end-2) = [vcircle,acircle];

        ROBOT_TRAJECTORY.state = ''; 

    case 'circle:_radial_heading'

        circumference = 2*pi*circle_radius^2;
        circle_speeds = circle_speed*ones(1,num_reps);
        times = circumference./circle_speeds;
        circle_times = zeros(1,2*length(circle_speeds));
        circle_times(2:2:end) = times;
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time hover_time  ... % up + hover
                                             out_time ... % to start
                                             circle_times...
                                             return_time land_time ]); % land
        num_static = 6;
        num_circles = (length(ROBOT_TRAJECTORY.timeline)-num_static)/2;  

        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},{'hovering'}...
                                      {'to_start'},...
              repmat([{'circle'},{'circle_radial_heading'}],1,num_circles)...                                                  
                                      {'hover_end'}...
                                      {'landing'},...                              
                                      {'landed.'} ];

        ROBOT_TRAJECTORY.waypoints(:,:,1:length(ROBOT_TRAJECTORY.behavior)) = ...
                                    zeros(4,3,length(ROBOT_TRAJECTORY.behavior));

        ROBOT_TRAJECTORY.waypoints(:,1,:) = ...
            repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), ... 0
                [0;0;hh;0], [0;0;hh;0], ... 2+5
                [0;circle_radius;hh;0], ... % to start of circle
                repmat([ [0;0;hh;0] ,... % 0 just to set center of circle
                         [0;circle_radius;hh;-2*pi] ], ... % 15: CIRCLE 
                         1,num_circles) ...         
                [0;0;hh;num_circles*-2*pi], ... % landing       
                [zeros(3,1);num_circles*-2*pi] ]; % land
        yaw_vec = reshape(repmat((1:num_circles)*-2*pi,2,1),1,(num_circles)*2);
        ROBOT_TRAJECTORY.waypoints(4,1,num_static+1:end-2) = yaw_vec(2:end-1);
                
        % velocity at start of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(4)+.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,4) = [vcircle,acircle];

        % velocity at end of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(end-2)-.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,end-2) = [vcircle,acircle];

        ROBOT_TRAJECTORY.state = ''; 

    case 'circle:_pirouette'
        circumference = 2*pi*circle_radius^2;
        circle_speeds = circle_speed*ones(1,num_reps);
        times = circumference./circle_speeds;
        circle_times = zeros(1,2*length(circle_speeds));
        circle_times(2:2:end) = times;
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time hover_time ... % 0 + up + hover
                                             out_time ... % to start
                                             circle_times...
                                             return_time land_time]); % land
        num_static = 6;
        num_circles = (length(ROBOT_TRAJECTORY.timeline)-num_static)/2;  

        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},{'hovering'}...
                                      {'to_start'},...
              repmat([{'circle'},{'circle_pirouette'}],1,num_circles)...                                                  
                                      {'hover_end'}...
                                      {'landing'},...                              
                                      {'landed.'} ];

        ROBOT_TRAJECTORY.waypoints(:,:,1:length(ROBOT_TRAJECTORY.behavior)) = ...
                                    zeros(4,3,length(ROBOT_TRAJECTORY.behavior));

        ROBOT_TRAJECTORY.waypoints(:,1,:) = ...
            repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), ... 0
                [0;0;hh;0], [0;0;hh;0], ... 2+5
                [0;circle_radius;hh;0], ... % to start of circle
                repmat([ [0;0;hh;0] ,... % 0 just to set center of circle
                         [0;circle_radius;hh;0] ], ... % 15: CIRCLE 
                         1,num_circles) ...         
                [0;0;hh;0], ... % landing       
                [zeros(3,1);0] ]; % land
        
        % setting yaw to repmat -6*pi means we want to yaw 3 times per
        % large circle
            foo = num_circles+1;
            bar = 1;  
        
        ns = num_spirals*2*pi;
        yaw_vec = reshape(repmat((1:foo)*-ns,2,1),1,[])+ns;
        ROBOT_TRAJECTORY.waypoints(4,1,4:end) = [yaw_vec(1:end) repmat(yaw_vec(end),1,bar)];
        
        % velocity at start of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(4)+.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,4) = [vcircle,acircle];

        % velocity at end of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(end-2)-.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,end-2) = [vcircle,acircle];

        ROBOT_TRAJECTORY.state = ''; 

    case 'spiral:_constant_heading'
        circumference = 2*pi*circle_radius^2;
%       life is easiest if we ask for an even number of circles        
        circle_speeds = .5*ones(1,num_reps);
        times = circumference./circle_speeds;
        circle_times = zeros(1,2*length(circle_speeds));
        circle_times(2:2:end) = times;
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time hover_time ... % up + hover
                                             out_time ... % to start
                                             circle_times...
                                             return_time land_time]); % land
        num_circles = length(circle_speeds);  

        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},{'hovering'}, {'to_start'},...
              repmat([{'circle'},{'circle_constant_heading'}],1,num_circles), ... 
              {'hover_end'}, {'landing'}, {'landed.'} ];

        ROBOT_TRAJECTORY.waypoints(:,:,1:length(ROBOT_TRAJECTORY.behavior)) = ...
                                    zeros(4,3,length(ROBOT_TRAJECTORY.behavior));

        ROBOT_TRAJECTORY.waypoints(:,1,:) = ...
            repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), [0;0;hh;0], [0;0;hh;0], ... 0+2+5
                [0;circle_radius;hh;0], ... % to start of circle
                repmat([ [0;0;hh;0] ,... % 0 just to set center of circle
                         [0;circle_radius;hh;-2*pi] ], ... % 15: CIRCLE 
                         1,num_circles), ...                    
                [0;0;hh;num_circles*-2*pi], ... % landing       
                [zeros(3,1);num_circles*-2*pi] ]; % land
            
%         yaw_vec = reshape(repmat((1:num_circles)*-2*pi,2,1),1,(num_circles)*2);
%         ROBOT_TRAJECTORY.waypoints(4,1,num_static+1:end-2) = yaw_vec(2:end-1);        
        % constant yaw
        ROBOT_TRAJECTORY.waypoints(4,1,:) = 0;
        
        % velocity at start of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(4)+.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,4) = [vcircle,acircle];

        % velocity at middle of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(4+(num_circles/2)*2+1)-.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,4+(num_circles/2)*2) = [vcircle,acircle];        
        
        % velocity at end of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(end-2)-.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,end-2) = [vcircle,acircle];
        
        % z heights & vels & accels during circles
        z_start = 4;
        z_peak = 4+num_circles;
        ROBOT_TRAJECTORY.waypoints(3,1,z_peak) = ROBOT_TRAJECTORY.waypoints(3,1,z_peak)+spiral_height;
        z_end = length(ROBOT_TRAJECTORY.timeline)-2;
        z_timeline_up = ROBOT_TRAJECTORY.timeline(z_start:z_peak);
        Z.waypoints = [ROBOT_TRAJECTORY.waypoints(3,:,[z_start z_peak])];
        Z.timeline = z_timeline_up(1,[1 end]);
        Z.behavior = repmat({'poly'},1,length(Z.timeline)+1);
        for i = 1:length(z_timeline_up)
            [pos_z,vel_z,acc_z,~] = set_desired(Z, z_timeline_up(i), circle_radius);
            ROBOT_TRAJECTORY.waypoints(3,1:3,z_start+i-1) = [pos_z,vel_z,acc_z];
        end
        z_timeline_down = ROBOT_TRAJECTORY.timeline(z_peak:z_end);
        Z.waypoints = [ROBOT_TRAJECTORY.waypoints(3,:,[z_peak z_end])];
        Z.timeline = z_timeline_down(1,[1 end]);
        Z.behavior = repmat({'poly'},1,length(Z.timeline)+1);
        for i = 1:length(z_timeline_down)
            [pos_z,vel_z,acc_z,~] = set_desired(Z, z_timeline_down(i), circle_radius);
            ROBOT_TRAJECTORY.waypoints(3,1:3,z_peak+i-1) = [pos_z,vel_z,acc_z];
        end        

        ROBOT_TRAJECTORY.state = ''; 

    case 'spiral:_radial_heading'
        circumference = 2*pi*circle_radius^2;
%       life is easiest if we ask for an even number of circles        
        circle_speeds = .5*ones(1,num_reps);
        times = circumference./circle_speeds;
        circle_times = zeros(1,2*length(circle_speeds));
        circle_times(2:2:end) = times;
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time hover_time ... % up + hover
                                             out_time ... % to start
                                             circle_times ...
                                             return_time land_time ]); % land
        num_circles = length(circle_speeds);  

        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},{'hovering'}, {'to_start'},...
              repmat([{'circle'},{'circle_constant_heading'}],1,num_circles), ... 
              {'hover_end'}, {'landing'}, {'landed.'} ];

        ROBOT_TRAJECTORY.waypoints(:,:,1:length(ROBOT_TRAJECTORY.behavior)) = ...
                                    zeros(4,3,length(ROBOT_TRAJECTORY.behavior));

        ROBOT_TRAJECTORY.waypoints(:,1,:) = ...
            repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), [0;0;hh;0], [0;0;hh;0], ... 0+2+5
                [0;circle_radius;hh;0], ... % to start of circle
                repmat([ [0;0;hh;0] ,... % 0 just to set center of circle
                         [0;circle_radius;hh;-2*pi] ], ... % 15: CIRCLE 
                         1,num_circles), ...                    
                [0;0;hh;num_circles*-2*pi], ... % landing       
                [zeros(3,1);num_circles*-2*pi] ]; % land
            
        num_static = 6;
        yaw_vec = reshape(repmat((1:num_circles)*-2*pi,2,1),1,(num_circles)*2);
        ROBOT_TRAJECTORY.waypoints(4,1,num_static+1:end-2) = yaw_vec(2:end-1);        
        
        % velocity at start of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(4)+.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,4) = [vcircle,acircle];

        % velocity at middle of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(4+(num_circles/2)*2+1)-.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,4+(num_circles/2)*2) = [vcircle,acircle];        
        
        % velocity at end of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(end-2)-.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,end-2) = [vcircle,acircle];
        
        % z heights & vels & accels during circles
        z_start = 4;
        z_peak = 4+num_circles;
        ROBOT_TRAJECTORY.waypoints(3,1,z_peak) = ROBOT_TRAJECTORY.waypoints(3,1,z_peak)+spiral_height;
        z_end = length(ROBOT_TRAJECTORY.timeline)-2;
        z_timeline_up = ROBOT_TRAJECTORY.timeline(z_start:z_peak);
        Z.waypoints = [ROBOT_TRAJECTORY.waypoints(3,:,[z_start z_peak])];
        Z.timeline = z_timeline_up(1,[1 end]);
        Z.behavior = repmat({'poly'},1,length(Z.timeline)+1);
        for i = 1:length(z_timeline_up)
            [pos_z,vel_z,acc_z,~] = set_desired(Z, z_timeline_up(i), circle_radius);
            ROBOT_TRAJECTORY.waypoints(3,1:3,z_start+i-1) = [pos_z,vel_z,acc_z];
        end
        z_timeline_down = ROBOT_TRAJECTORY.timeline(z_peak:z_end);
        Z.waypoints = [ROBOT_TRAJECTORY.waypoints(3,:,[z_peak z_end])];
        Z.timeline = z_timeline_down(1,[1 end]);
        Z.behavior = repmat({'poly'},1,length(Z.timeline)+1);
        for i = 1:length(z_timeline_down)
            [pos_z,vel_z,acc_z,~] = set_desired(Z, z_timeline_down(i), circle_radius);
            ROBOT_TRAJECTORY.waypoints(3,1:3,z_peak+i-1) = [pos_z,vel_z,acc_z];
        end        

        ROBOT_TRAJECTORY.state = ''; 
        
    case 'spiral:_pirouette'
        circumference = 2*pi*circle_radius^2;
%       life is easiest if we ask for an even number of circles        
        circle_speeds = .5*ones(1,num_reps);
        times = circumference./circle_speeds;
        circle_times = zeros(1,2*length(circle_speeds));
        circle_times(2:2:end) = times;
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time hover_time ... % up + hover
                                             out_time ... % to start
                                             circle_times...
                                             return_time land_time ]); % land
        num_circles = length(circle_speeds);  

        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},{'hovering'}, {'to_start'},...
              repmat([{'circle'},{'circle_constant_heading'}],1,num_circles), ... 
              {'hover_end'}, {'landing'}, {'landed.'} ];

        ROBOT_TRAJECTORY.waypoints(:,:,1:length(ROBOT_TRAJECTORY.behavior)) = ...
                                    zeros(4,3,length(ROBOT_TRAJECTORY.behavior));

        ROBOT_TRAJECTORY.waypoints(:,1,:) = ...
            repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), [0;0;hh;0], [0;0;hh;0], ... 0+2+5
                [0;circle_radius;hh;0], ... % to start of circle
                repmat([ [0;0;hh;0] ,... % 0 just to set center of circle
                         [0;circle_radius;hh;0] ], ... % 15: CIRCLE 
                         1,num_circles), ...                    
                [0;0;hh;num_circles*0], ... % landing       
                [zeros(3,1);num_circles*0] ]; % land
            
        % setting yaw to repmat -6*pi means we want to yaw 3 times per
        % large circle
            foo = num_circles+1;
            bar = 1;  
       
        ns = num_spirals*2*pi;            
        yaw_vec = reshape(repmat((1:foo)*-ns,2,1),1,[])+ns;
        ROBOT_TRAJECTORY.waypoints(4,1,4:end) = [yaw_vec(1:end) repmat(yaw_vec(end),1,bar)];
        
        
        % velocity at start of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(4)+.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,4) = [vcircle,acircle];

        % velocity at middle of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(4+(num_circles/2)*2+1)-.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,4+(num_circles/2)*2) = [vcircle,acircle];        
        
        % velocity at end of circles
        [~,vcircle,acircle,~] = set_desired(ROBOT_TRAJECTORY, ROBOT_TRAJECTORY.timeline(end-2)-.0001,circle_radius);
        ROBOT_TRAJECTORY.waypoints(:,2:3,end-2) = [vcircle,acircle];
        
        % z heights & vels & accels during circles
        z_start = 4;
        z_peak = 4+num_circles;
        ROBOT_TRAJECTORY.waypoints(3,1,z_peak) = ROBOT_TRAJECTORY.waypoints(3,1,z_peak)+spiral_height;
        z_end = length(ROBOT_TRAJECTORY.timeline)-2;
        z_timeline_up = ROBOT_TRAJECTORY.timeline(z_start:z_peak);
        Z.waypoints = [ROBOT_TRAJECTORY.waypoints(3,:,[z_start z_peak])];
        Z.timeline = z_timeline_up(1,[1 end]);
        Z.behavior = repmat({'poly'},1,length(Z.timeline)+1);
        for i = 1:length(z_timeline_up)
            [pos_z,vel_z,acc_z,~] = set_desired(Z, z_timeline_up(i), circle_radius);
            ROBOT_TRAJECTORY.waypoints(3,1:3,z_start+i-1) = [pos_z,vel_z,acc_z];
        end
        z_timeline_down = ROBOT_TRAJECTORY.timeline(z_peak:z_end);
        Z.waypoints = [ROBOT_TRAJECTORY.waypoints(3,:,[z_peak z_end])];
        Z.timeline = z_timeline_down(1,[1 end]);
        Z.behavior = repmat({'poly'},1,length(Z.timeline)+1);
        for i = 1:length(z_timeline_down)
            [pos_z,vel_z,acc_z,~] = set_desired(Z, z_timeline_down(i), circle_radius);
            ROBOT_TRAJECTORY.waypoints(3,1:3,z_peak+i-1) = [pos_z,vel_z,acc_z];
        end        

        ROBOT_TRAJECTORY.state = ''; 
        
    case 'pirouette:_spin-in-place'
        % time per behavior, including time to travel from start -> hover-start ->
        % behavior start, and behavior-end > hover-end -> land
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time hover_time ...
                                            1 ...    ramp-up
                                             pirouette_time*ones(1,num_reps) ...   spin
                                            1 ...   ramp-down
                                             land_time ]); % land
        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},...
                                      {'hover_start'},...
                                      {'ramp_up'},...
                                      repmat({'spin_in_place'},1,num_reps),...                                     
                                      {'deaccelerate'},...
                                      {'landing'},...                              
                                      {'landed.'} ];

       ROBOT_TRAJECTORY.waypoints(:,:,1:length(ROBOT_TRAJECTORY.behavior)) = ...
                                    zeros(4,3,length(ROBOT_TRAJECTORY.behavior));

        spins = repmat([0;0;hh;0],1,num_reps);
        spins(4,:) = 3*pi:2*pi:2*pi*num_reps+pi;

        ROBOT_TRAJECTORY.waypoints(:,1,:) = ...
           repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), ... start
                [0;0;hh;0], ... up
                [0;0;hh;0], ... hover
                [0;0;hh;pi], ... ramp up
                spins ... spins
                [0;0;hh;spins(4,end)+pi], ... ramp down
                [zeros(3,1);spins(4,end)+pi] ]; % land

        ROBOT_TRAJECTORY.state = ''; 

        % find velocity for ramp-up
        [~,vel_des,acc_des,~] = set_desired(ROBOT_TRAJECTORY, ...
                ROBOT_TRAJECTORY.timeline(4)+.0001, circle_radius);
        ROBOT_TRAJECTORY.waypoints(4,2:3,4) = [vel_des(4),acc_des(4)];

        % find velocity for ramp-down
        [~,vel_des,acc_des,~] = set_desired(ROBOT_TRAJECTORY, ...
                ROBOT_TRAJECTORY.timeline(end-2)-.0001, circle_radius);
        ROBOT_TRAJECTORY.waypoints(4,2:3,end-2) = [vel_des(4),acc_des(4)];

    case 'square:_constant_heading'
        square_fixed_heading.waypoints  = [ 1   1    -1   -1       1 ; ...
                                            1  -1    -1    1       1 ; ...
                                            hh   hh   hh   hh     hh ; ...
                                            0   0     0    0       0 ];        
        square_fixed_heading.waypoints(4,:) = 45*pi/180*ones(1,length(square_fixed_heading.waypoints));
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time ... % up 
                                             out_time ... % to start
                                             3   3   3   3 ... % square fixed heading                                     
                                             return_time land_time ]); % land
        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},...
                                      {'square_start'},...
                                      repmat({'square_fixed_heading'},1,size(square_fixed_heading.waypoints,2))...                                                           
                                      {'hovering'},...
                                      {'landing'},...                              
                                      ];
        ROBOT_TRAJECTORY.waypoints = repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), ...
                [0;0;hh;square_fixed_heading.waypoints(4,1)], ...
                square_fixed_heading.waypoints, ...        
                [0;0;hh;square_fixed_heading.waypoints(4,end)], ... % hover-land
                [zeros(3,1);0] ]; % land
        ROBOT_TRAJECTORY.state = ''; 

    case 'square:_turn_corners'
    square_turn_corners.waypoints   = [ 1   1   1    -1    -1   -1   -1        1       1; ...
                                        1  -1  -1    -1    -1    1    1        1       1; ...
                                        hh  hh  hh   hh    hh   hh    hh       hh     hh ; ...
                                        0   0   pi/2  pi/2  pi   pi   3*pi/2   3*pi/2  2*pi ];    
        ROBOT_TRAJECTORY.timeline = cumsum([ 0 rise_time ... % up 
                                             out_time ... % to start
                                             4 2 4 2 4 2 4 2 ... % square turn corners                                     
                                             return_time land_time]); % land
        ROBOT_TRAJECTORY.behavior = [ {'takeoff'},...
                                      {'hovering'},...
                                      repmat({'square_turn_corners'},1,size(square_turn_corners.waypoints,2))...                                                                                                                
                                      {'hovering'},...
                                      {'landing'},...                              
                                      ];
        ROBOT_TRAJECTORY.waypoints = repmat(start_waypoint,1,length(ROBOT_TRAJECTORY.behavior)) + ...
              [ zeros(4,1), ...
                [0;0;hh;0], ...
                square_turn_corners.waypoints, ...         
                [0;0;hh;square_turn_corners.waypoints(4,end)], ... % hover-land
                [zeros(3,1);square_turn_corners.waypoints(4,end)]]; % land
        ROBOT_TRAJECTORY.waypoints(4,:) = -ROBOT_TRAJECTORY.waypoints(4,:);    
        ROBOT_TRAJECTORY.state = ''; 
                        
    otherwise
        display('behavior not recognized')
end


end
%% plotting for debugging
%{
figure()
plot(ROBOT_TRAJECTORY.timeline,squeeze(ROBOT_TRAJECTORY.waypoints(3,1,:)))
hold on
for i = 1:length(ROBOT_TRAJECTORY.timeline)
text(ROBOT_TRAJECTORY.timeline(i),ROBOT_TRAJECTORY.waypoints(3,1,i),num2str(i))
end
plot(ROBOT_TRAJECTORY.timeline,squeeze(ROBOT_TRAJECTORY.waypoints(4,1,:)),'r')
%}