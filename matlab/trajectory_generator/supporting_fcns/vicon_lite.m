function [ODOM] = vicon_lite(ODOM, tstart, odom_msg)

%% decompose the structure for use
odom_idx = ODOM.odom_idx;
% odom_headerT = ODOM.odom_headerT;
% pos_odom = ODOM.pos_odom;
% vel_odom = ODOM.vel_odom;
% acc_odom = ODOM.acc_odom;
% jrk_odom = ODOM.jrk_odom;
% att_odom = ODOM.att_odom;
% ang_odom = ODOM.ang_odom;
myodom = ODOM.myodom;
% pos_odom_cuttoff = ODOM.pos_odom_cuttoff;
% vel_odom_cuttoff = ODOM.vel_odom_cuttoff;
% acc_odom_cuttoff = ODOM.acc_odom_cuttoff;
% jrk_odom_cuttoff = ODOM.jrk_odom_cuttoff;
% att_odom_cuttoff = ODOM.att_odom_cuttoff;
% ang_odom_cuttoff = ODOM.ang_odom_cuttoff;
%%
% smooth = 3;

% if odom_idx <= smooth
       
%     odom_headerT(odom_idx) = odom_msg.header.stamp;

%     pos_odom(:,odom_idx) = [odom_msg.pose.pose.position.x;
%                             odom_msg.pose.pose.position.y;
%                             odom_msg.pose.pose.position.z];
%     vel_odom(:,odom_idx) = [odom_msg.twist.twist.linear.x;
%                             odom_msg.twist.twist.linear.y;
%                             odom_msg.twist.twist.linear.z];
    myodom.pos = [odom_msg.pose.pose.position.x;
                            odom_msg.pose.pose.position.y;
                            odom_msg.pose.pose.position.z];
    myodom.vel = [odom_msg.twist.twist.linear.x;
                            odom_msg.twist.twist.linear.y;
                            odom_msg.twist.twist.linear.z];
%     if odom_idx-1 > 0 
%         dt = odom_headerT(odom_idx)-odom_headerT(odom_idx-1);
%         acc_odom(:,odom_idx) = ...
%             (vel_odom(:,odom_idx)-vel_odom(:,odom_idx-1)) / ...
%             dt;
%         jrk_odom(:,odom_idx) = ...
%             (acc_odom(:,odom_idx)-acc_odom(:,odom_idx-1)) / ...
%             dt;
%     else
%         acc_odom(:,odom_idx) = zeros(3,1);
%         jrk_odom(:,odom_idx) = zeros(3,1);
%     end

%     att_odom(:,odom_idx) = ...
%                 geometry_utils.RToZYX(geometry_utils.QuatToR(odom_msg.pose.pose.orientation));
% 
%     ang_odom(:,odom_idx) = [odom_msg.twist.twist.angular.x; ...
%                            odom_msg.twist.twist.angular.y; ...
%                            odom_msg.twist.twist.angular.z];      

    myodom.att = ...
                geometry_utils.RToZYX(geometry_utils.QuatToR(odom_msg.pose.pose.orientation));

    myodom.ang = [odom_msg.twist.twist.angular.x; ...
                           odom_msg.twist.twist.angular.y; ...
                           odom_msg.twist.twist.angular.z];      
                       
                       
%     pos_odom_cuttoff(:,odom_idx) = pos_odom(:,odom_idx);
%     vel_odom_cuttoff(:,odom_idx) = vel_odom(:,odom_idx);
%     att_odom_cuttoff(:,odom_idx) = att_odom(:,odom_idx);
%     ang_odom_cuttoff(:,odom_idx) = ang_odom(:,odom_idx);
%     acc_odom_cuttoff(:,odom_idx) = acc_odom(:,odom_idx);
%     jrk_odom_cuttoff(:,odom_idx) = jrk_odom(:,odom_idx);
%     
%     myodom.pos = mean(pos_odom(:,odom_idx),2);            
%     myodom.vel = mean(vel_odom(:,odom_idx),2);           
%     myodom.att = mean(att_odom(:,odom_idx),2);
%     myodom.ang = mean(ang_odom(:,odom_idx),2);
%     myodom.acc = mean(acc_odom(:,1:odom_idx),2);
%     myodom.jrk = mean(jrk_odom(:,1:odom_idx),2);       
  
    myodom.acc = zeros(3,1);%acc_odom(:,1:odom_idx);
    myodom.jrk = zeros(3,1);%jrk_odom(:,1:odom_idx);
    %{
else
    id1 = smooth;
    id0 = 2;
    odom_headerT = [ odom_headerT(id0:id1) odom_msg.header.stamp];
    dt = odom_headerT(end)-odom_headerT(end-1);

    pos_odom = horzcat(pos_odom(:,id0:id1), [odom_msg.pose.pose.position.x;
                                             odom_msg.pose.pose.position.y;
                                             odom_msg.pose.pose.position.z]);
    vel_odom = horzcat(vel_odom(:,id0:id1), [odom_msg.twist.twist.linear.x;
                                             odom_msg.twist.twist.linear.y;
                                             odom_msg.twist.twist.linear.z]);
    acc = (vel_odom(:,end)-vel_odom(:,end-1))/dt;
    acc_odom = [ acc_odom(:,id0:id1), acc];
    
    jrk = (acc_odom(:,end)-acc_odom(:,end)-1)/dt;
    jrk_odom = [ jrk_odom(:,id0:id1), jrk ];

    att = geometry_utils.RToZYX(geometry_utils.QuatToR(odom_msg.pose.pose.orientation));
    att_odom = [att_odom(:,id0:id1) att];
        
    ang_odom = horzcat(ang_odom(:,id0:id1), [odom_msg.twist.twist.angular.x; ...
                                           odom_msg.twist.twist.angular.y; ...
                                           odom_msg.twist.twist.angular.z]);            

    % rough cutoffs...
    % dt = .01
    dpos = .10; % vel_max = 10m/s^2, then dp_max = .1m
%     dvel = .05; % acc_max = 5m/s^2, then dv_max = .05m/s
%     datt = .05;
%     dang = 2;
%     dacc = 5*10^2;
%     djrk = 5*10^3;

    if sum(abs(pos_odom(:,end) - myodom.pos)) < dpos
        pos_odom_cuttoff = [pos_odom_cuttoff(:,id0:id1) pos_odom(:,end)];
        vel_odom_cuttoff = [vel_odom_cuttoff(:,id0:id1) vel_odom(:,end)];
        att_odom_cuttoff = [att_odom_cuttoff(:,id0:id1) att_odom(:,end)];
        ang_odom_cuttoff = [ang_odom_cuttoff(:,id0:id1) ang_odom(:,end)];
        acc_odom_cuttoff = [acc_odom_cuttoff(:,id0:id1) acc_odom(:,end)];
    else
        pos_odom_cuttoff = [pos_odom_cuttoff(:,id0:id1) pos_odom_cuttoff(:,end)];
        vel_odom_cuttoff = [vel_odom_cuttoff(:,id0:id1) vel_odom_cuttoff(:,end)];
        att_odom_cuttoff = [att_odom_cuttoff(:,id0:id1) att_odom_cuttoff(:,end)];
        ang_odom_cuttoff = [ang_odom_cuttoff(:,id0:id1) ang_odom_cuttoff(:,end)];
        acc_odom_cuttoff = [acc_odom_cuttoff(:,id0:id1) acc_odom_cuttoff(:,end)];
    end

    myodom.pos = mean(pos_odom_cuttoff,2);
    myodom.vel = mean(vel_odom_cuttoff,2);            
    myodom.att = mean(att_odom_cuttoff,2);
    myodom.ang = mean(ang_odom_cuttoff,2);
    myodom.acc = mean(acc_odom_cuttoff,2);
    myodom.jrk = mean(jrk_odom_cuttoff,2);  
       
end
%}
odom_idx = odom_idx + 1;
odom_updated = true;

if odom_idx == 2
  ODOM.start_pos = myodom.pos;
end
        
%% rebuild the structure to send back
ODOM.odom_idx = odom_idx;
% ODOM.odom_headerT = odom_headerT;
% ODOM.pos_odom = pos_odom;
% ODOM.vel_odom = vel_odom;
% ODOM.acc_odom = acc_odom;
% ODOM.jrk_odom = jrk_odom;
% ODOM.att_odom = att_odom;
% ODOM.ang_odom = ang_odom;
ODOM.myodom = myodom;
% ODOM.pos_odom_cuttoff = pos_odom_cuttoff;
% ODOM.vel_odom_cuttoff = vel_odom_cuttoff;
% ODOM.acc_odom_cuttoff = acc_odom_cuttoff;
% ODOM.jrk_odom_cuttoff = jrk_odom_cuttoff;
% ODOM.att_odom_cuttoff = att_odom_cuttoff;
% ODOM.ang_odom_cuttoff = ang_odom_cuttoff;
ODOM.odom_updated = odom_updated;

end