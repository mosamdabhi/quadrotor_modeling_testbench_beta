function [noisyLinAcc, noisyAngVel] = sensor_noise(linAcc, angVel)
N = 3;
mu = 0;            
sigma = 0;            
% normally distributed with mean=mu and std=sigma
noiseLinAcc = randn(N, 1)*sigma + mu;  
noiseAngVel = randn(N, 1)*sigma + mu;
noisyLinAcc = linAcc + noiseLinAcc;
noisyAngVel = angVel + noiseAngVel;
end