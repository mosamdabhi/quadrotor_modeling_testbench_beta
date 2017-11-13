function [robot, IB, checkIB, t_end] = shutdown( elapsed_t, ...
                    max_accel, max_vel, robot, ODOM, IMU, skid  )


display(['shutdown at ' num2str(elapsed_t) ' seconds'])

% set waypoints to land @ STARTING position
startpos1 = robot.waypoints(:,1,1);
    if ndims(robot.waypoints)<3
        startpos2 = robot.waypoints(:,2);
    else
        startpos2 = robot.waypoints(:,1,2);
    end
% startpos2 = [startpos1(1:2); 1];
% small check to make a robot stay landed if it is
% already landed
if (ODOM.myodom.pos(3) < .2) && ...
   (sum(abs(ODOM.myodom.vel) < .01) == 3)
   startpos2 = [ODOM.myodom.pos(1:2); startpos1(3)];
   startpos1 = startpos2;
end
% erase & establish new waypoints
robot.waypoints = [];
%%
acc = zeros(4,1);
if (ODOM.odom_idx-1) < 12
    foo = 1; 
else
    foo = ODOM.odom_idx-1-10;
end
bar = ODOM.odom_idx -1;
avg_vel = mean(ODOM.vel_odom(:,foo:bar),2);
avg_yang = mean(ODOM.ang_vel(:,foo:bar),2);
robot.waypoints(:,:,1) = [  [ODOM.myodom.pos; ODOM.myodom.att(3)], ...
    [avg_vel; avg_yang(3)], acc ];

%%

robot.waypoints(:,:,1) = [[ODOM.myodom.pos, avg_vel, zeros(3,1)]; ...
    ODOM.myodom.att(3), avg_yang(3), 0];

% these times & positions are just estimates
tf = skid.dmin_t; %max(abs(ODOM.myodom.vel)/max_accel)+.5; % +n for buffer
xf = avg_vel.*tf + ODOM.myodom.pos;%ODOM.myodom.vel.*tf + ODOM.myodom.pos;
robot.waypoints(:,:,2) = [[xf, zeros(3,2)]; zeros(1,3)];
tend1 = tf+.5;%4;

robot.waypoints(:,:,3) = [[startpos2(1:3) zeros(3,2)]; zeros(1,3)];
D = max(diag(pdist2(robot.waypoints(1:3,1,2),robot.waypoints(1:3,1,3))));                    
tend2 = find_tf(D,max_vel,max_accel); 
% tend2 = 4;                    

robot.waypoints(:,:,4) = [startpos1 zeros(4,2)];                                   
D = max(diag(pdist2(robot.waypoints(1:3,1,3),robot.waypoints(1:3,1,4))));                    
tend3 = find_tf(D,max_vel,max_accel); 
% tend3 = 4;

% set timeline
robot.timeline = [elapsed_t, elapsed_t+tend1, elapsed_t+tend1+tend2, elapsed_t+tend1+tend2+tend3];


% set new end time
t_end = 0;
if robot.timeline(end) > t_end 
    t_end = robot.timeline(end);        
end
   
% make it so we don't accept new instructions while we land
IB = 0;
checkIB = false;     

end
