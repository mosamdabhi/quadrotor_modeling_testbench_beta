%% Nonlinear 6DOF dynamics for a quadrotor
% Dynamics model like QuadrotorDynamics.m with CG offset from geometric
% center included

function [F] = QuadrotorDynamics(t, W, QuadModel, desRPM)
%% Initialize F and params
F = zeros(22,1);

%% U after hard constraints on current RPM
[U] = QuadModel.constrained_control_inputs(W(13:16));

%% Rotational Dynamics
tau = U(2:4);
angVel = W(4:6);
F(4:6) = QuadModel.AngularDynamics(tau, angVel, U);

%% Translational dynamics in world frame
F(1:3) = QuadModel.TranslationalDynamics(W, U, F(4:6));

%% Linear velocities in world frame
F(7:9) = W(1:3);

%% Euler Rates
rotMatrix = [ 1  sin(W(10))*tan(W(11))  cos(W(10))*tan(W(11));           
                      0              cos(W(10))             -sin(W(10));         
                      0  sin(W(10))/cos(W(11))  cos(W(10))/cos(W(11));];
F(10:12) = rotMatrix*W(4:6);

%% Motor dynamics
F(13:16) = -QuadModel.kMotor*(W(13:16) - desRPM');

%% Noisy IMU data calculation
[F(17:19), noisyAngVel] = sensor_noise(F(1:3), F(4:6));

%% Linear velocities with noise
F(20:22) = W(17:19);

%% Hard constraint on floor (Detecting floor)
% if(W(9) < zOffset) 
%     F(1:12) = zeros(12,1);
%     F(17:26) = zeros(10,1);
% end
end