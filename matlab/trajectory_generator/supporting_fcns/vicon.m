function [ODOM] = vicon(ODOM, tstart, odom_msg)

%% decompose the structure for use
odom_idx = ODOM.odom_idx;
odom_headerT = ODOM.odom_headerT;
odom_time = ODOM.odom_time;
pos_odom = ODOM.pos_odom;
vel_odom = ODOM.vel_odom;
acc_odom = ODOM.acc_odom;
jrk_odom = ODOM.jrk_odom;
att_odom = ODOM.att_odom;
ang_odom = ODOM.ang_odom;
myodom = ODOM.myodom;
pos_odom_cuttoff = ODOM.pos_odom_cuttoff;
vel_odom_cuttoff = ODOM.vel_odom_cuttoff;
acc_odom_cuttoff = ODOM.acc_odom_cuttoff;
jrk_odom_cuttoff = ODOM.jrk_odom_cuttoff;
att_odom_cuttoff = ODOM.att_odom_cuttoff;
ang_odom_cuttoff = ODOM.ang_odom_cuttoff;
pos_odom_smoothed = ODOM.pos_odom_smoothed;
vel_odom_smoothed = ODOM.vel_odom_smoothed;
acc_odom_smoothed = ODOM.acc_odom_smoothed;
jrk_odom_smoothed = ODOM.jrk_odom_smoothed;
att_odom_smoothed = ODOM.att_odom_smoothed;
ang_odom_smoothed = ODOM.ang_odom_smoothed;

%%
odom_time(odom_idx) = toc(tstart);
odom_headerT(odom_idx) = odom_msg.header.stamp;

pos_odom(:,odom_idx) = [odom_msg.pose.pose.position.x;
                        odom_msg.pose.pose.position.y;
                        odom_msg.pose.pose.position.z];
vel_odom(:,odom_idx) = [odom_msg.twist.twist.linear.x;
                        odom_msg.twist.twist.linear.y;
                        odom_msg.twist.twist.linear.z];
if odom_idx-1 > 0 
    dt = odom_headerT(odom_idx)-odom_headerT(odom_idx-1);
    acc_odom(:,odom_idx) = ...
        (vel_odom(:,odom_idx)-vel_odom(:,odom_idx-1)) / ...
        dt;
    jrk_odom(:,odom_idx) = ...
        (acc_odom(:,odom_idx)-acc_odom(:,odom_idx-1)) / ...
        dt;
else
    acc_odom(:,odom_idx) = zeros(3,1);
    jrk_odom(:,odom_idx) = zeros(3,1);
end

att_odom(:,odom_idx) = ...
            geometry_utils.RToZYX(geometry_utils.QuatToR(odom_msg.pose.pose.orientation));

ang_odom(:,odom_idx) = [odom_msg.twist.twist.angular.x; ...
                       odom_msg.twist.twist.angular.y; ...
                       odom_msg.twist.twist.angular.z];                                                                               
%{
smooth = 3;
if odom_idx > smooth
% rough cutoffs...
dpos = .10; % vel_max = 10m/s^2, then dp_max = .1m
% dvel = .05; % acc_max = 5m/s^2, then dv_max = .05m/s
% datt = .05;
% dang = 2;
% dacc = 5*10^2;
% djrk = 5*10^3;

    if sum(abs(pos_odom(:,odom_idx) - pos_odom_smoothed(:,odom_idx-1))) < dpos
        pos_odom_cuttoff(:,odom_idx) = pos_odom(:,odom_idx);
        vel_odom_cuttoff(:,odom_idx) = vel_odom(:,odom_idx);
        att_odom_cuttoff(:,odom_idx) = att_odom(:,odom_idx);
        ang_odom_cuttoff(:,odom_idx) = ang_odom(:,odom_idx);
        acc_odom_cuttoff(:,odom_idx) = acc_odom(:,odom_idx);
    else
        pos_odom_cuttoff(:,odom_idx) = pos_odom_cuttoff(:,odom_idx-1);
        vel_odom_cuttoff(:,odom_idx) = vel_odom_cuttoff(:,odom_idx-1);
        att_odom_cuttoff(:,odom_idx) = att_odom_cuttoff(:,odom_idx-1);
        ang_odom_cuttoff(:,odom_idx) = ang_odom_cuttoff(:,odom_idx-1);
        acc_odom_cuttoff(:,odom_idx) = acc_odom_cuttoff(:,odom_idx-1);
    end
%{    
    if sum(abs(vel_odom(:,odom_idx) - vel_odom_cuttoff(:,odom_idx-1))) < dvel
        vel_odom_cuttoff(:,odom_idx) = vel_odom(:,odom_idx);
    else
        vel_odom_cuttoff(:,odom_idx) = vel_odom_cuttoff(:,odom_idx-1);
    end
    
    if sum(abs(att_odom(:,odom_idx) - att_odom_cuttoff(:,odom_idx-1))) < datt
        att_odom_cuttoff(:,odom_idx) = att_odom(:,odom_idx);
    else
        att_odom_cuttoff(:,odom_idx) = att_odom_cuttoff(:,odom_idx-1);
    end
    
    if sum(abs(ang_odom(:,odom_idx) - ang_odom_cuttoff(:,odom_idx-1))) < dang
        ang_odom_cuttoff(:,odom_idx) = ang_odom(:,odom_idx);
    else
        ang_odom_cuttoff(:,odom_idx) = ang_odom_cuttoff(:,odom_idx-1);
    end
    
    if sum(abs(acc_odom(:,odom_idx) - acc_odom_cuttoff(:,odom_idx-1))) < dacc
        acc_odom_cuttoff(:,odom_idx) = acc_odom(:,odom_idx);
    else
        acc_odom_cuttoff(:,odom_idx) = acc_odom_cuttoff(:,odom_idx-1);
    end
    
    if sum(abs(jrk_odom(:,odom_idx) - jrk_odom_cuttoff(:,odom_idx-1))) < djrk
        jrk_odom_cuttoff(:,odom_idx) = jrk_odom(:,odom_idx);
    else
        jrk_odom_cuttoff(:,odom_idx) = jrk_odom_cuttoff(:,odom_idx-1);
    end
%}
    myodom.pos = mean(pos_odom_cuttoff(:,odom_idx-smooth:odom_idx),2);
    myodom.vel = mean(vel_odom_cuttoff(:,odom_idx-smooth:odom_idx),2);            
    myodom.att = mean(att_odom_cuttoff(:,odom_idx-smooth:odom_idx),2);
    myodom.ang = mean(ang_odom_cuttoff(:,odom_idx-smooth:odom_idx),2);
    myodom.acc = mean(acc_odom_cuttoff(:,odom_idx-smooth:odom_idx),2);
    myodom.jrk = mean(jrk_odom_cuttoff(:,odom_idx-smooth:odom_idx),2);    
   
else
    pos_odom_cuttoff(:,odom_idx) = pos_odom(:,odom_idx);
    vel_odom_cuttoff(:,odom_idx) = vel_odom(:,odom_idx);
    att_odom_cuttoff(:,odom_idx) = att_odom(:,odom_idx);
    ang_odom_cuttoff(:,odom_idx) = ang_odom(:,odom_idx);
    acc_odom_cuttoff(:,odom_idx) = acc_odom(:,odom_idx);
    jrk_odom_cuttoff(:,odom_idx) = jrk_odom(:,odom_idx);
    
    myodom.pos = mean(pos_odom(:,odom_idx),2);            
    myodom.vel = mean(vel_odom(:,odom_idx),2);           
    myodom.att = mean(att_odom(:,odom_idx),2);
    myodom.ang = mean(ang_odom(:,odom_idx),2);
    myodom.acc = mean(acc_odom(:,1:odom_idx),2);
    myodom.jrk = mean(jrk_odom(:,1:odom_idx),2);    
end

% cuttoff check           
pos_odom_smoothed(:,odom_idx) = myodom.pos;
vel_odom_smoothed(:,odom_idx) = myodom.vel;
acc_odom_smoothed(:,odom_idx) = myodom.acc;
jrk_odom_smoothed(:,odom_idx) = myodom.jrk;
att_odom_smoothed(:,odom_idx) = myodom.att;
ang_odom_smoothed(:,odom_idx) = myodom.ang;
%}
                       
myodom.pos = pos_odom(:,odom_idx);
myodom.vel = vel_odom(:,odom_idx);
myodom.att = att_odom(:,odom_idx);
myodom.ang = ang_odom(:,odom_idx);
myodom.acc = acc_odom(:,odom_idx);
myodom.jrk = jrk_odom(:,odom_idx);

odom_idx = odom_idx + 1;
odom_updated = true;

if odom_idx == 2
  ODOM.start_pos = myodom.pos;
end
        
%% rebuild the structure to send back
ODOM.odom_idx = odom_idx;
ODOM.odom_headerT = odom_headerT;
ODOM.odom_time = odom_time;
ODOM.pos_odom = pos_odom;
ODOM.vel_odom = vel_odom;
ODOM.acc_odom = acc_odom;
ODOM.jrk_odom = jrk_odom;
ODOM.att_odom = att_odom;
ODOM.ang_odom = ang_odom;
ODOM.myodom = myodom;
ODOM.pos_odom_cuttoff = pos_odom_cuttoff;
ODOM.vel_odom_cuttoff = vel_odom_cuttoff;
ODOM.acc_odom_cuttoff = acc_odom_cuttoff;
ODOM.jrk_odom_cuttoff = jrk_odom_cuttoff;
ODOM.att_odom_cuttoff = att_odom_cuttoff;
ODOM.ang_odom_cuttoff = ang_odom_cuttoff;
ODOM.pos_odom_smoothed = pos_odom_smoothed;
ODOM.vel_odom_smoothed = vel_odom_smoothed;
ODOM.acc_odom_smoothed = acc_odom_smoothed;
ODOM.jrk_odom_smoothed = jrk_odom_smoothed;
ODOM.att_odom_smoothed = att_odom_smoothed;
ODOM.ang_odom_smoothed = ang_odom_smoothed;
ODOM.odom_updated = odom_updated;

end