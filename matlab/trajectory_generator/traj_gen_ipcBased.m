function [pos_des, vel_des, acc_des, jrk_des] =  traj_gen_ipcBased(odom)
% Input the odom data of pos and att obtained from vicon
% This is supposed to be run as a live script so the update loops are run
% at real time
% Outputs: Desired state

addpath('supporting_fcns');
sim_flag = true;
run_lite = false;
circle_radius = 0.75;

% set up data structures
if run_lite
    samples = 1;
    big_samples = 200000;
else
    samples = 10000;
    big_samples = samples;
end

% structure to hold trajectory information for the robot
VIRTUAL_LEADER_TRAJECTORY = struct();
VIRTUAL_LEADER_TRAJECTORY.timeline = 0; 
VIRTUAL_LEADER_TRAJECTORY.behavior = {};
VIRTUAL_LEADER_TRAJECTORY.waypoints = zeros(4,1);
VIRTUAL_LEADER_TRAJECTORY.state = ' ';
VIRTUAL_LEADER_TRAJECTORY.Rfcn = 0;
VIRTUAL_LEADER_TRAJECTORY.Sfcn = 0;
VIRTUAL_LEADER_TRAJECTORY.S = zeros(3, 1);

state_transition.idx = 0;
state_transition.time = 0;
state_transition.state = {};


% set starting waypoints based on Vicon

ODOM.myodom.pos = odom.pos % Random values as of now. Take the actual present odom pos values here and att values in the below command
ODOM.myodom.att = odom.att; 

start_waypoint = [ ODOM.myodom.pos; ODOM.myodom.att(3) ];    
r=1;
VIRTUAL_LEADER_TRAJECTORY.S(:,r) = start_waypoint(1:3);

% use vicon & define trajecotries (structs taken from Ellen's baseline performance - use the ref later when we add various behaviors)
str = {'hover', 'line_x', 'line_y', ...
       'up-down', ...
       'up-down:_pirouette', ...
       'circle:_constant_heading'...
       'circle:_radial_heading', ...
       'circle:_pirouette', ...
       'spiral:_constant_heading', ...
       'spiral:_radial_heading', ...
       'spiral:_pirouette', ...
       'pirouette:_spin-in-place', ...
       'square:_constant_heading', ...
       'square:_turn_corners'};
[s,v] = listdlg('PromptString','Select a trajectory:',...
                'SelectionMode','single','ListSize',[300 300],...
                'ListString',str);
if v
    trajectory = str{s};
else
    display('no behavior selected'); %break (use this break when we are inside a while loop eventually)
end

leader_start_waypoint = [ mean(VIRTUAL_LEADER_TRAJECTORY.S(:,:,1),2); 0 ]; % computing mean of each row
robot_traj = retrieve_trajectory(trajectory,leader_start_waypoint,circle_radius);
VIRTUAL_LEADER_TRAJECTORY.timeline = robot_traj.timeline; 
VIRTUAL_LEADER_TRAJECTORY.behavior = robot_traj.behavior;
VIRTUAL_LEADER_TRAJECTORY.waypoints = robot_traj.waypoints;
VIRTUAL_LEADER_TRAJECTORY.S = VIRTUAL_LEADER_TRAJECTORY.S - repmat(leader_start_waypoint(1:3),1,1);

loop_dt = nan(1,big_samples);
loop_time = nan(1,big_samples);
lasttic = tic;
dtidx = 1;
xcon_pre = 0;
ycon_pre = 0;
zcon_pre = 0;

t_end = VIRTUAL_LEADER_TRAJECTORY.timeline(end);

% Control loop
tstart = tic;
while toc(tstart) < t_end  
    elapsed_t = toc(tstart); 
    
% display state
    timeline = VIRTUAL_LEADER_TRAJECTORY.timeline;
    passed_times_idx = elapsed_t>=timeline;
    last_waypoint_idx = sum(passed_times_idx);
    if (last_waypoint_idx >= length(VIRTUAL_LEADER_TRAJECTORY.timeline))
        last_waypoint_idx = length(VIRTUAL_LEADER_TRAJECTORY.timeline);
    end
    last_state = VIRTUAL_LEADER_TRAJECTORY.state;
    state = VIRTUAL_LEADER_TRAJECTORY.behavior(last_waypoint_idx);
    if ~strcmp(last_state,state)
        VIRTUAL_LEADER_TRAJECTORY.state = state;
        display( [num2str(elapsed_t) ' ' state{:} ] );
        display(' ')
        state_transition.idx = state_transition.idx + 1;
        state_transition.time(state_transition.idx) = elapsed_t;
        state_transition.state(state_transition.idx) = state;
    end
    
    % measuring the overall control loop time
    dt = toc(lasttic);
    lasttic = tic;
    loop_dt(dtidx) = dt;
    loop_time(dtidx) = toc(tstart);
    dtidx = dtidx + 1;
    
    % get leader traj and offset it to find our robot trajectory
    
    [Cpos_des, Cvel_des, Cacc_des, Cjrk_des] = ...
        set_desired(VIRTUAL_LEADER_TRAJECTORY, elapsed_t, circle_radius);
    if Cpos_des(4) > pi || Cpos_des(4) < -pi
        Cpos_des(4) = mod(Cpos_des(4),sign(Cpos_des(4))*2*pi);
        if Cpos_des(4) > pi
           Cpos_des(4) = Cpos_des(4) - 2*pi;
        elseif Cpos_des(4) < -pi
           Cpos_des(4) = Cpos_des(4) + 2*pi;
        end
    end
    
    C = repmat(Cpos_des(1:3),1,1);
    Cdot = repmat(Cvel_des(1:3),1,1);
    Cddot = repmat(Cacc_des(1:3),1,1);
    Cdddot = repmat(Cjrk_des(1:3),1,1);
    
    thEta = 0;
    thEta_d = 0;
    thEta_dd = 0;
    thEta_ddd = 0;
    
     R =    [  cos(thEta)  -sin(thEta) 0 ; ...
               sin(thEta)   cos(thEta) 0 ; ...
               0            0          1 ];          
     Rdot = skew([0;0;thEta_d])*R; 
     Rddot = skew([0;0;thEta_dd])*R + skew([0;0;thEta_d])*Rdot;
     Rdddot = skew([0;0;thEta_ddd])*R + skew([0;0;thEta_dd])*Rdot + ...
              skew([0;0;thEta_dd])*Rdot + skew([0;0;thEta_d])*Rddot;            
     
     S = VIRTUAL_LEADER_TRAJECTORY.S;
     Sdot = zeros(size(S));
     Sddot = zeros(size(S));
     Sdddot = zeros(size(S));
     
     Pos_Des = R*S + C;
     Vel_Des = Rdot*S + R*Sdot + Cdot;
     Acc_Des = Rddot*S + 2*Rdot*Sdot + R*Sddot + Cddot; 
     Jrk_Des = Rdddot*S + Rddot*Sdot +...
               2*Rddot*Sdot + 2*Rdot*Sddot + ...
               Rdot*Sddot + R*Sdddot + ...
               Cdddot; 

    % retrieve desired set point in position or attitude 

    pos_des = Pos_Des(:,1);
    vel_des = Vel_Des(:,1);
    acc_des = Acc_Des(:,1);
    jrk_des = Jrk_Des(:,1);
end

     
     

        
