function [IMU] = imu(IMU, tstart, imu_msg)

%% decompose the structure for use
imu_time = IMU.imu_time;
imu_headerT = IMU.imu_headerT;
imu_idx = IMU.imu_idx;
att_imu = IMU.att_imu;
ang_imu = IMU.ang_imu;
myimu = IMU.myimu;

%%
imu_time(:,imu_idx) = toc(tstart);
imu_headerT(imu_idx) = imu_msg.header.stamp;

att_imu(:,imu_idx) = ...
            geometry_utils.RToZYX(geometry_utils.QuatToR(imu_msg.orientation));

ang_imu(:,imu_idx) = [imu_msg.angular_velocity.x; ...
                      imu_msg.angular_velocity.y; ...
                      imu_msg.angular_velocity.z];

% rather than remember what index we're on, just make a temp
% variable representing the most current imu-based attitude            
myimu.att = att_imu(:,imu_idx);
myimu.ang = ang_imu(:,imu_idx);

imu_idx = imu_idx + 1;
imu_updated = true;           

%% rebuild the structure to send back
IMU.imu_idx = imu_idx;
IMU.imu_time = imu_time;
IMU.imu_headerT = imu_headerT;
IMU.att_imu = att_imu;
IMU.ang_imu = ang_imu;
IMU.myimu = myimu;
IMU.imu_updated = imu_updated;

end