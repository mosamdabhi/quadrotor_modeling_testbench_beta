function [mean_, cov_] = kalman_update(mean, cov, control, measurement, sys)
% Kalman filter update for a controller linear dynamical system.

%% Predicted belief (not accounting for the latest measurement)
meanTemp = sys.A * mean + sys.B * control;
covTemp = sys.A * cov * (sys.A)' + sys.R;

%% Kalman gain calculation
G = covTemp * (sys.C)' * inv((sys.C) * covTemp * (sys.C)' + sys.Q);

%% Correction
mean_ = meanTemp + G * (measurement - (sys.C * meanTemp));
cov_ = (eye(2) - (G * sys.C)) * covTemp;
end
