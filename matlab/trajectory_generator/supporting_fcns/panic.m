function [robot, IB, checkIB, t_end, NOMORE] = panic( elapsed_t, ...
                    robot, send_rpm, rpm_msg, rpm_pub, cd_pub, Kp_att, Kd_att )

display(['E-stopped at ' num2str(elapsed_t) ' seconds'])

% send 0 msg
if send_rpm
    rpm_cmd = zeros(1,4);
    rpm_msg.motor_rpm = rpm_cmd;
    rpm_pub.publish(rpm_msg);
else
    cd_msg = cd_pub.empty();
    cd_msg.header.stamp = elapsed_t;
    cd_msg.thrust = 0;
        q_out.x = 0;
        q_out.y = 0;
        q_out.z = 0;
        q_out.w = 1;
    cd_msg.orientation = q_out;
        av_out.x = 0;
        av_out.y = 0;
        av_out.z = 0;
    cd_msg.angular_velocity = av_out;
        kr = Kp_att;
        Kr.x = kr(1); Kr.y = kr(5); Kr.z = kr(9);
    cd_msg.kR = Kr;
        ko = Kd_att;
        kOm.x = ko(1); kOm.y = ko(5); kOm.z = ko(9);            
    cd_msg.kOm = kOm;
    cd_pub.publish(cd_msg);
end
% set end time in the past, to make really sure we never
% come back here
t_end = -1;
IB = 0;
checkIB = false;
NOMORE = true;
