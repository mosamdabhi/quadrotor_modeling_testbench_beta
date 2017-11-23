clear;
clc;
close all;

addpath('../sensor_models/DataLog/');
addpath('../sensor_models/');
addpath('../classes');
addpath('../utilities');

load('time.mat');

hulk1c = QuadrotorModel;
[a, av, omega, hulk1c] = simulate_IMU(hulk1c);

%% Vector initialization
W = zeros(10,length(omega));
W(10, 1) = 1;
W(3,1) = 2;
W(1,1) = 0;

%% ODE 45
for i = 1:1:length(omega)-1
augW = W(:,i);
tspan = [t(i) t(i+1)];
[t_soln, W_soln] = ode45(@(t, augW) continuous_time(t, augW, a(i,:)', omega(i,:)', hulk1c), tspan, augW);
W(:,i+1) =  W_soln(end,1:10)';
disp(t(i));
end

states = vertcat(t(1:length(W(1,:)))', W);

figure;
grid on;
subplot(3,1,1);plot(states(1,:),states(2,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
subplot(3,1,2);plot(states(1,:),states(3,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
subplot(3,1,3);plot(states(1,:),states(4,:), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
set(gca,'fontsize',12,'FontWeight','bold');

figure;
plot3(states(2,:), states(3,:), states(4,:), 'LineWidth', 1.5);
axis([-4 4 -4 4 0 5]);
grid on;
xlabel('X', 'FontSize', 20);
ylabel('Y', 'FontSize', 20);
zlabel('Z', 'FontSize', 20);
set(gca,'fontsize',12,'FontWeight','bold');
view(3);