function [pos_des,vel_des,acc_des,jrk_des] = set_desired(robot, elapsed_t, circle_radius)

timeline = robot.timeline;
passed_times_idx = elapsed_t>=timeline;

last_waypoint_idx = sum(passed_times_idx);
next_waypoint_idx = last_waypoint_idx+1;

if last_waypoint_idx == length(timeline)
   last_waypoint_idx = length(timeline); 
   next_waypoint_idx = last_waypoint_idx;
end

start_time = timeline(last_waypoint_idx);
if ndims(robot.waypoints)<3
    last_waypoint = robot.waypoints(:,last_waypoint_idx);
else
    last_waypoint = robot.waypoints(:,:,last_waypoint_idx);
end

end_time = timeline(next_waypoint_idx);
if ndims(robot.waypoints)<3
    next_waypoint = robot.waypoints(:,next_waypoint_idx);
else
    next_waypoint = robot.waypoints(:,:,next_waypoint_idx);
end

timescale = end_time - start_time;
myt = elapsed_t-start_time;

if last_waypoint_idx == length(timeline)
    % some dummy times to keep us hovering at a position if we have passed
    % in a timeline with no more moves.
    timescale = 10;
    myt = 1;
end

behavior = robot.behavior{last_waypoint_idx};
if length(behavior)<6
    behavior = [behavior repmat(' ',1,6-length(behavior))];
end

if strcmp(behavior(1:6),'circle')
    [pos_xy, vel_xy, acc_xy, jrk_xy ] = circle_xy(myt, timescale, last_waypoint(1:2,1), circle_radius);
%     [pos_psi,vel_psi,acc_psi,jrk_psi] = poly5(myt,timescale,last_waypoint(4,:),next_waypoint(4,:));
    vel_psi = (next_waypoint(4,1)-last_waypoint(4,1))/timescale;
    if ndims(robot.waypoints)<3
        old_psi = robot.waypoints(4,last_waypoint_idx);
    else
        old_psi = robot.waypoints(4,1,last_waypoint_idx);
    end
    pos_psi = vel_psi*myt + old_psi;
    acc_psi = 0;
    jrk_psi = 0;
    [pos_z,vel_z,acc_z,jrk_z] = poly5(myt,timescale,last_waypoint(3,:),next_waypoint(3,:));

    pos_des = [ pos_xy; pos_z; pos_psi ];
    vel_des = [ vel_xy; vel_z; vel_psi ];
    acc_des = [ acc_xy; acc_z; acc_psi ];
    jrk_des = [ jrk_xy; jrk_z; jrk_psi ];  
    
elseif strcmp(behavior(1:4),'spin')
    [pos_xyz, vel_xyz, acc_xyz, jrk_xyz ] = poly5(myt,timescale,last_waypoint(1:3,:,:),next_waypoint(1:3,:,:));
    
    vel_psi = (next_waypoint(4,1)-last_waypoint(4,1))/timescale;
    if ndims(robot.waypoints)<3
        old_psi = robot.waypoints(4,last_waypoint_idx);
    else
        old_psi = robot.waypoints(4,1,last_waypoint_idx);
    end
    pos_psi = vel_psi*myt + old_psi;
    acc_psi = 0;
    jrk_psi = 0;

    pos_des = [ pos_xyz; pos_psi ];
    vel_des = [ vel_xyz; vel_psi ];
    acc_des = [ acc_xyz; acc_psi ];
    jrk_des = [ jrk_xyz; jrk_psi ];      
else
    % each "waypoint" has the form: 
    %               1st col = pos,   2nd col = vel,   3rd col = acc
    % 1st row = x
    % 2nd row = y
    % 3rd row = z
    % 4th row = psi
    [pos_des,vel_des,acc_des,jrk_des] = poly5(myt,timescale,last_waypoint,next_waypoint);
end    

end
