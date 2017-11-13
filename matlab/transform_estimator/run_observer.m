clear;
clc;
close all;

%% Add neccessary paths
addpath('../sensor_models');
addpath('../sensor_models/DataLog');
addpath('../classes');
addpath('../kalman_filter'); 
addpath('../utilities');
addpath('../solvers');

load('time.mat');
load('lin_vel.mat');

%% Call estimator for the whole time duration
% flags
plot_flag = true; % No plotting on the first call.
% Call estimate_transforms to produce filter results in the struct OUTPUT_.
OUTPUT_ = estimate_transforms(t(end-1, 1), plot_flag);

% % Find global minima
% [peaks, locs] = findpeaks(-OUTPUT_.cost,t(1:length(OUTPUT_.cost)));
% [value, minidx] = max(peaks);
% optimalTime = locs(minidx);
% 
% %% Call estimator for the minima time location
% plot_flag = true;
% OUTPUT_FINAL = estimate_transforms(optimalTime, plot_flag);
% 
% %% Delete MAT files containing sensor data.
% rmdir('../sensor_models/DataLog', 's');
