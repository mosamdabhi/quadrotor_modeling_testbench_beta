# SE(3) parameter estimation.
## Software system components.
1. Large angle controller : Formulated as in D. Mellinger's [thesis](http://repository.upenn.edu/cgi/viewcontent.cgi?article=1705&context=edissertations). Navigate to `/controllers` for the code. 
2. Trajectory generator : Time parameterized trajectories. Defined in `/trajectory_generator/traj_params.m`.

You can run 1 and 2 by calling `quad_dyn_main`.

3. SE(3) transform estimator : Run `transform_determination` to use it. It will first run dynamics to generate sensor data (based on your trajectory and time selection) and would produce plots pertaining to estimator. 

## Parameter definitions
1. Quadrotor related params are currently in the `QuadrotorModel` class in `/classes`.
2. IMU to Vicon transformation can be set in `/sensor_models/simulate_IMU.m`.
3. Accelerometer and gyroscope noise can be adjusted in `/sensor_models/sensor_noise.m`.

Reference:
Mirzaei F. and Roumeliotis S., A Kalman Filter-Based Algorithm for IMU-Camera Calibration: Observability Analysis and Performance Evaluation, IEEE Transactions on Robotics, Volume 24 Issue 5

