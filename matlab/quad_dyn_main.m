%% Quadrotor Simulation for performance characterization
%  X : 12X1 state vector
%     vx = X(1);  Global frame velocity in x
%     vy = X(2);  Global frame velocity in y
%     vz = X(3);  Global frame velocity in z
%     p = X(4);  Body frame angular velocity in x
%     q = X(5);  Body frame angular velocity in y
%     r = X(6);  Body frame angular velocity in z
%     xw = X(7);  Global X coordinate 
%     yw = X(8);  Global Y coordinate 
%     zw = X(9);  Global Z coordinate 
%     phi = X(10); Roll angle
%     theta = X(11); Pitch angle
%     psi = X(12); Yaw angle
%      w1 = X(13); RPM 1
%      w2 = X(14); RPM 2
%      w3 = X(15); RPM 3
%      w4 = X(16); RPM 4
%     
%  xNoise : 6x1 vector for noisy IMU output (Zero mean gaussian noise)
%     noisy vx = X(17);
%     noisy vy = X(18);
%     noisy vz = X(19);
%     noisy xw = X(20);
%     noisy yw = X(21);
%     noisy zw = X(22);
%
% U : Desired RPMs X(23:26)

function quad_dyn_main(actualTheta)

%% Add paths to required folders
addpath('sensor_models');
addpath('classes');
addpath('utilities');
addpath('controllers');
addpath('solvers');
addpath('trajectory_generator');
addpath('transform_estimator');

%% Initialize state vector, vehicle object and trajectory selection
if actualTheta == 1
    hulk1c = QuadrotorModel;
    theta_actual = [hulk1c.cT; hulk1c.cTorque; 0; diag(hulk1c.I); hulk1c.rOffset];
else
    hulk1c = QuadrotorModel2;
    theta_bar = [hulk1c.cT; hulk1c.cTorque; 0; diag(hulk1c.I); hulk1c.rOffset];
end


if actualTheta == 1
    save('../matlab/transform_estimator/Mat_file_logs/theta_actual.mat', 'theta_actual');
else
    save('../matlab/transform_estimator/Mat_file_logs/theta_bar.mat', 'theta_bar');
end

prompt = 'End time (in seconds) : ';
t_final = input(prompt);
prompt = 'Dynamics solver time step : ';
h = input(prompt);
prompt = 'Which Solver? 0 for RK4, 1 for ODE45 : ';
solver = input(prompt);
str = {'Circle', ...
       'Leminiscate', ...
       'Eight Curve', ...
       'Ellipse', ...
       'Cornoid', ...
       'Astroid', ...
       'Vertical Circle', ...
       'Bicorn', ...
       'Butterfly Curve', ...
       'TakeOff', ...
       'Spiral Circle', ...
       'Circle_MultiDimensional', ...
       'Back&Forth'};
[s,v] = listdlg('PromptString','Select a trajectory:',...
                'SelectionMode','single','ListSize',[300 300],...
                'ListString',str);
if v
    trajectory = str{s};
else
    display('no behavior selected')
    return
end
X = init_state(trajectory);

timespan = 0:h:t_final;

%% GUI Call
%gui_main;

%% Initialize vectors required in simulation
RPM = zeros(4,1);
xNoise = zeros(6,1);
desRPM = zeros(4,1);
t(1) = 0;
timeVec(1) = 0;
W(:,1) = [X; RPM; xNoise];

innerLoopData = zeros(length(timespan)-1, 13);
outerLoopData = zeros(length(timespan)-1, 13);
rpm = zeros(length(timespan)-1, 4);
quat_save = zeros(t_final/h,4);
ang_vel_save = zeros(t_final/h, 3);
lin_ang_accel_save = zeros(t_final/h, 6);

%% Controller specific initialization
% Controller gains
controller.kp = [4.0; 10.0; 16.0];
controller.kv = [5.5; 6.5; 10.0];

controller.kR = [10.0; 10.0; 2.0]; 
controller.kOm = [0.60; 0.60; 0.1];

%% Simulation Loop
if solver == 0
t = zeros(length(timespan)-1, 1);
tic; % Start simulation time
for i = 1:(length(timespan)-1)
   % Display time
   disp(t(i));
   
   %% Large angle control    
    % Error calculation (Calls trajectory generator and publishes error value)
    error = error_calculation(W(:,i), t(i), trajectory);
        
    %time_save(i,1) = t(i);
    
    % Current quaternion calculation
    q = ZYXToQuat(W(10:12,i));
    [angCommands, u1Des] = large_angle_position_controller(error, q, controller, hulk1c);
    
    quat_save(i,:) = [q.x,q.y,q.z,q.w];
    
    % Attitude error calculation
    attErr = attitude_error_calculation(W(:,i), angCommands, q, t(i));
    
    % Calling inner loop attitude controller
    [uNoConstraint] = large_angle_attitude_controller(attErr, u1Des, controller);
    
    % RPM calculation
    [rpm(i,:)] = hulk1c.convert_bodyForces_toRPM(uNoConstraint);
    
    % Store data for post processing
    outerLoopData(i,:) = error.outerLoopData;
    innerLoopData(i,:) = attErr.innerLoopData;  
    
    U(:, i) = hulk1c.constrained_control_inputs(W(13:16));
   % solve dynamics numerically using 4th order Runge-Kutta Method
   [W(:,i+1), lin_ang_accel_save(i,:)] = solve_using_rk4(t(i), W(:,i), hulk1c, rpm(i,:), h);
   
   ang_vel_save(i,:) = W(4:6,i+1);
   
   % update time
   t(i+1) = t(i) + h;  
end
toc; % End simulation time
lin_vel_store = W(1:3,:);
%% Saving MAT files
if actualTheta == 1
    mkdir ../matlab/sensor_models/DataLog;
    save ('../matlab/sensor_models/DataLog/quat_save.mat', 'quat_save');
    save ('../matlab/sensor_models/DataLog/ang_vel_save.mat', 'ang_vel_save');
    save ('../matlab/sensor_models/DataLog/lin_ang_accel_save.mat', 'lin_ang_accel_save');
    save ('../matlab/sensor_models/DataLog/time.mat', 't');
    save ('../matlab/sensor_models/DataLog/lin_vel.mat', 'lin_vel_store');
    save ('../matlab/sensor_models/DataLog/FTotalMTotal.mat', 'U');
end

%% Converting to different state variable
for i = 1:length(W(10:12,:))
    new_q = ZYXToQuat(W(10:12,i));
    new_quat(1,i) = new_q.x;
    new_quat(2,i) = new_q.y;
    new_quat(3,i) = new_q.z;
    new_quat(4,i) = new_q.w;
    new_rB(1:3,i) = W(7:9,i);
    new_vB(1:3,i) = W(1:3,i);
    new_omegaB(1:3,i) = W(4:6,i);
    new_rpm(1:4,i) = W(13:16,i);
    new_noise(1:6,i) = W(17:22,i);
end
new_W = vertcat(new_rB, new_vB, new_quat, new_omegaB, new_noise, new_rpm);

if actualTheta == 1
    measurement_Z = vertcat(new_rB, new_vB, new_quat);
    save('../matlab/transform_estimator/Mat_file_logs/measurement_Z.mat', 'measurement_Z');
    save('../matlab/transform_estimator/Mat_file_logs/U_rpm_MAV.mat', 'rpm');
else
    save('../matlab/transform_estimator/Mat_file_logs/X_bar.mat', 'new_W');
end
    
Y_forInnerLoop = vertcat(t', W);
Y_forInnerLoop = Y_forInnerLoop';
Y = vertcat(t', new_W);
Y = Y';
else
    
tic; % Start simulation time
for i = 1:(length(timespan)-1)
   % Display time
   disp(t(i));
   
   %% Large angle control    
    % Error calculation (Calls trajectory generator and publishes error value)
    error = error_calculation(W(:,end), t(end));
    
    % Current quaternion calculation
    q = ZYXToQuat(W(10:12,end));
    [angCommands, u1Des] = large_angle_position_controller(error, q, controller, hulk1c);
    
    % Attitude error calculation
    attErr = attitude_error_calculation(W(:,end), angCommands, q, t(end));
    
    % Calling inner loop attitude controller
    [uNoConstraint] = large_angle_attitude_controller(attErr, u1Des, controller);
    
    % RPM calculation
    [rpm(i,:)] = hulk1c.convert_bodyForces_toRPM(uNoConstraint);
    
    % Store data for post processing
    outerLoopData(i,:) = error.outerLoopData;
    innerLoopData(i,:) = attErr.innerLoopData;

   % solve dynamics numerically using 4th order Runge-Kutta Method
   tspan = [t(i) t(i)+h];
   [t_soln, W_soln] = ode45(@(t,W) QuadrotorDynamics(t, W(:,end), hulk1c, rpm(i,:)), tspan, W(:,end));
   W = [W, W_soln'];
   timeVec = [timeVec, t_soln'];
   
   % update time
   t(i+1) = t(i) + h;  
end
toc; % End simulation time

for ii = 1:(length(timespan)-1)
    q = W(7:10, ii);
    W(7:9, ii) = QuatToZYX(q);
    W(10:22, ii) = W(11:23, ii);
end

Y = vertcat(timeVec, W);
Y = Y';
end

%% Plot results
plot_inner_loop_errors(innerLoopData, Y_forInnerLoop);
plot_outer_loop_errors(outerLoopData, Y);
plot_trajectory_data(outerLoopData, Y);

plot_rpm(innerLoopData, rpm);
end