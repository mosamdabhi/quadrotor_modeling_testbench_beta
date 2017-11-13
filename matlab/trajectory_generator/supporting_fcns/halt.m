function [robot, IB, checkIB, t_end] = halt( elapsed_t, ...
                    max_accel, speeds, robot, ODOM, IMU  )
                                        
display(['halted at ' num2str(elapsed_t) ' seconds'])

% set waypoints to land @ CURRENT position
robot.waypoints = [];
robot.waypoints(:,:,1) = [[ODOM.myodom.pos, ODOM.myodom.vel, zeros(3,1)]; ...
                           ODOM.myodom.att(3), IMU.myimu.ang(3), 0];

% these times & positions are just estimates
tf = max(abs(ODOM.myodom.vel)/max_accel)+.5; % +n for buffer
xf = ODOM.myodom.vel.*tf + ODOM.myodom.pos;
robot.waypoints(:,:,2) = [[xf, zeros(3,2)]; zeros(1,3)];
tend1 = tf+.5;

robot.waypoints(:,:,3) = [[[xf(1:2);0] zeros(3,2)]; zeros(1,3)];
D = max(diag(pdist2(robot.waypoints(:,1,2),robot.waypoints(:,1,3))));                    
tend2 = find_tf(D,speeds(2),max_accel); 
% tend2 = 4;                    

% set timeline
robot.timeline = [elapsed_t, elapsed_t+tend1, elapsed_t+tend1+tend2];


% set new end time
t_end = 0;
if robot.timeline(end) > t_end 
    t_end = robot.timeline(end);        
end
   
% make it so we don't accept new instructions while we land
IB = 0;
checkIB = false;
