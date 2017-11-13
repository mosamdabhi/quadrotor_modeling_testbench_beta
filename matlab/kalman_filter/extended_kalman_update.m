function [mean_, cov_] = extended_kalman_update(mean, cov, control, mRes, sys)
% extended kalman filter update for a controller linear dynamical system.

%% Predicted belief (not accounting for the latest measurement)
meanTemp = sys.g(control, mean);
covTemp = sys.G * cov * (sys.G)' + sys.R;

%% Kalman gain calculation
K = covTemp * (sys.H)' * inv((sys.H) * covTemp * (sys.H)' + sys.Q);

%% Correction
mean_ = meanTemp + K * (mRes);
cov_ = (eye(2) - (K * sys.H)) * covTemp;
end