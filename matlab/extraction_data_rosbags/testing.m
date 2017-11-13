%% Add the control_arch_test_suite path here. 
addpath('/home/mosam/repositories/control_arch_test_suite/matlab');
%% Select following topics by running `cats` (All these topics are structures internally): 
%../vehicleName/imu; 
%../vehicleName/motion_manager/rpm_cmd;
%../vehicleName/motion_manager/tracking_error;
%../vehicleName/odom
cats;
%% Extract the data in clusters(strcutures)
imu_cluster = bagfile_data.danaus05.imu;
rpm_cmd = bagfile_data.danaus05.motion_manager.rpm_cmd;
odom_cluster = bagfile_data.danaus05.odom;
%% Save the data in Datalog folder
save ('Datalog/imu_cluster.mat','imu_cluster');
save ('Datalog/rpm_cmd.mat','rpm_cmd');
save ('Datalog/odom_cluster.mat','odom_cluster');