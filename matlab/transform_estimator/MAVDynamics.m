%% Nonlinear 6DOF dynamics for a quadrotor
% Dynamics model like QuadrotorDynamics.m with CG offset from geometric
% center included

function [F] = MAVDynamics(t, W, QuadModel, desRPM)
%% Initialize F and params
F = zeros(23,1);

%% U after hard constraints on current RPM
[U] = QuadModel.constrained_control_inputs(W(20:23, 1));

%% Rotational Dynamics
tau = U(2:4);
angVel = W(11:13, 1);
F(11:13) = QuadModel.AngularDynamics(tau, angVel, U);

%% Translational dynamics in world frame
F(4:6) = QuadModel.TranslationalDynamics(W, U, F(11:13));

%% Linear velocities in world frame
F(1:3) = W(4:6, 1);

%% Quaternion update
capitalOmega = [-skewmat(angVel), angVel;
                        -angVel',        0];
F(7:10,1) = 0.5.*(capitalOmega*W(7:10,1));   

%% Bias terms are not updated in MAV dynamics
F(14:16) = zeros(3,1);
F(17:19) = zeros(3,1);

%% Motor dynamics
F(20:23) = -QuadModel.kMotor*(W(20:23,1) - desRPM');

%% Hard constraint on floor (Detecting floor)
% if(W(9) < zOffset) 
%     F(1:12) = zeros(12,1);
%     F(17:26) = zeros(10,1);
% end
end