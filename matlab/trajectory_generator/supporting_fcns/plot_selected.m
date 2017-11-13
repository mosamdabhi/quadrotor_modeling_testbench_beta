function [] = plot_selected(ToPlot,ROBOT_HISTORY,ROBOT_TRAJECTORY,IMU,ODOM,looptime,behavior)
rgbk = [ 255 0 0; 34 139 34; 0 0 255; 0 0 0 ]/255;  
gray = ones(1,3)*170/255;
lt_gray = ones(1,3)*245/255;
pastel_red = [255 225 225]/255; 
pastel_green = [200 255 200]/255;
pastel_blue = [220 250 250]/255;
pastel_black = [220 220 220]/255;   

if iscell(behavior)
    behavior = behavior{:};
end
underscores = strfind(behavior,'_');
temp = '';
if ~isempty(underscores)
   for i = 1:length(underscores)
       if i==1
           foo = 1;
       else
           foo = underscores(i-1)    ;     
       end
       bar = underscores(i)-1;
       temp = strcat(temp,behavior(foo:bar));
       temp = strcat(temp,'\');
   end
   temp = strcat(temp,behavior(underscores(end):end));
end
if ~isempty(temp)
    behavior = temp;
end

%% convert q_des to roll&pitch des
idx = ~isnan(ROBOT_HISTORY.store_qdes(1,:));
ROBOT_HISTORY.phi_des = nan(1,length(idx));
ROBOT_HISTORY.theta_des = nan(1,length(idx));
ROBOT_HISTORY.psi_des = nan(1,length(idx));

    Q = ROBOT_HISTORY.store_qdes(:,idx);

% now we can convert Q to Euler angles
    q0 = Q(1,:);
    q1 = Q(2,:);
    q2 = Q(3,:);
    q3 = Q(4,:);

    ROBOT_HISTORY.phi_des(1:length(Q))   = atan( 2*(q0.*q1 + q2.*q3) ./ (1 - 2*(q1.*q1 + q2.*q2)) );
    ROBOT_HISTORY.theta_des(1:length(Q)) = asin( 2*(q0.*q2 - q3.*q1) );
    ROBOT_HISTORY.psi_des(1:length(Q))   = atan2( 2*(q0.*q3 + q1.*q2) , (1 - 2*(q2.*q2 + q3.*q3)) );
%{
% load('volume_flight1.mat')
Idx = ~isnan(ROBOT_HISTORY.store_qdes(1,:));
ROBOT_HISTORY.phi_des = nan(1,length(Idx));
ROBOT_HISTORY.theta_des = nan(1,length(Idx));
ROBOT_HISTORY.psi_des = nan(1,length(Idx));
% this is a large operation, handle in chunks to avoid mem errors. :(
allQ = nan(4,length(Idx));
div = 100;
for portion = 0:div:length(Idx)  
    sidx = portion+1;
    if portion+div<length(Idx)
        eidx = portion+div;
    else     
        eidx = length(Idx);        
    end
    idx = Idx(sidx:eidx);    

    Q_trimmed = ROBOT_HISTORY.store_qdes(:,idx);
% ROBOT_HISTORY.store_qdes is the trimmed q_des. For comparison to any
% measurement made of tne robot, we want the untrimmed q_des
    % qTrimsInv = [trimsQ(1);-trimsQ(2:4)]/sqrt(sum(trimsQ.^2));
    % untrimmed = quatmult(qTrimsInv,q_trimmed);
    Q_trimmed_inv = [Q_trimmed(1,:); -Q_trimmed(2:4,:)];% magnitude is 1 already    
    a = Q_trimmed;          %A = reshape(a,[4,1,length(idx)]);
    AA = zeros(4*length(idx),length(idx));
    AAidx = (1:4*length(idx))+reshape(repmat((0:4*length(idx):4*length(idx)^2-1),4,1),1,4*length(idx));
    AA(AAidx) = a(:);
    
    b = Q_trimmed_inv;      B = nan(4,4,length(idx));   
    B(1,1,:) = b(1,:);      B(1,2,:) = -b(2,:);     B(1,3,:) = -b(3,:);     B(1,4,:) = -b(4,:);
    B(2,1,:) = b(2,:);      B(2,2,:) =  b(1,:);     B(2,3,:) =  b(4,:);     B(2,4,:) = -b(3,:);
    B(3,1,:) = b(3,:);      B(3,2,:) = -b(4,:);     B(3,3,:) =  b(1,:);     B(3,4,:) =  b(2,:);
    B(4,1,:) = b(4,:);      B(4,2,:) =  b(3,:);     B(4,3,:) = -b(2,:);     B(4,4,:) =  b(1,:);
    B = reshape(B,4,[]);
    B = B(:);
    % Q = [ b(1)   -b(2)    -b(3)   -b(4); ...
    %       b(2)    b(1)     b(4)   -b(3); ...
    %       b(3)   -b(4)     b(1)    b(2); ...
    %       b(4)    b(3)    -b(2)    b(1) ]*a;
    BB = zeros(4*length(idx));
%     foo = reshape(repmat((0:length(idx)*4^2:(4*length(idx))^2-1),4^2,1),4,[])';
%     bar = repmat(0:4*length(idx):4*length(idx)*(4-1),length(idx)*4,1);
%     br = repmat((1:4*length(idx))',1,4);
%     bc = foo+bar;
%     BBidx = br+bc;
    bc = repmat(0:4*length(idx):(length(idx)*4)^2-1,4,1);
        foo = reshape(repmat((0:4:4*length(idx)-1),4^2,1),4,[]);
        bar = repmat((1:4)',1,4*length(idx));
    br = foo+bar;
    BBidx = br+bc;    
    BB(BBidx) = B;
    Q = BB*AA;
    Q = reshape(Q(AAidx),4,[]);
    allQ(:,sidx:eidx) = Q;
end
Q = allQ;
clear foo bar br bc a A AA AA idx b B BB BBidx Q_trimmed Q_trimmed_inv allQ

% now we can convert Q to Euler angles
    q0 = Q(1,:);
    q1 = Q(2,:);
    q2 = Q(3,:);
    q3 = Q(4,:);

    ROBOT_HISTORY.phi_des(1:length(Q))   = atan( 2*(q0.*q1 + q2.*q3) ./ (1 - 2*(q1.*q1 + q2.*q2)) );
    ROBOT_HISTORY.theta_des(1:length(Q)) = asin( 2*(q0.*q2 - q3.*q1) );
    ROBOT_HISTORY.psi_des(1:length(Q))   = atan2( 2*(q0.*q3 + q1.*q2) , (1 - 2*(q2.*q2 + q3.*q3)) );
%}
%%    
for n = 1:length(ToPlot)
    name = ToPlot{n};
    switch name
        case ' '
            % do nothing, blank string
            
        case 'loop_time'
            h=figure();
            endl = sum(~isnan(looptime));
                plot(1:endl,looptime(1:endl),'marker','.')
                title('loop time')
                xlabel('loop iteration')
                ylabel('time [s]')
            figure(h)  

        case 'position'
            h = figure();
            subplot(2,2,1)
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.store_posDes(1,:),'color',gray); hold on;
                plot(ODOM.odom_time, ODOM.pos_odom(1,:),'color',rgbk(1,:),'marker','.','linestyle','none') 
%                 plot(ODOM.odom_time, ODOM.pos_odom_smoothed(1,:),'color',rgbk(4,:)) 
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])
                title('x')
                ax_y = get(gca,'ylim');
            subplot(2,2,2)
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.store_posDes(2,:),'color',gray); hold on;
                plot(ODOM.odom_time, ODOM.pos_odom(2,:),'color',rgbk(2,:),'marker','.','linestyle','none') 
%                 plot(ODOM.odom_time, ODOM.pos_odom_smoothed(2,:),'color',rgbk(4,:)) 
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])      
                title('y')
                ax_y = get(gca,'ylim');
            subplot(2,2,3)
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.store_posDes(3,:),'color',gray); hold on;
                plot(ODOM.odom_time, ODOM.pos_odom(3,:),'color',rgbk(3,:),'marker','.','linestyle','none') 
%                 plot(ODOM.odom_time, ODOM.pos_odom_smoothed(3,:),'color',rgbk(4,:)) 
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])  
                title('z')
                ax_y = get(gca,'ylim');
            subplot(2,2,4)
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.store_posDes(4,:),'color',gray); hold on;
                plot(ODOM.odom_time, ODOM.att_odom(3,:),'color',rgbk(4,:),'marker','.','linestyle','none') 
%                 plot(ODOM.odom_time, ODOM.att_odom_smoothed(3,:),'color',rgbk(4,:)) 
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])      
                title('yaw')
                ax_y = get(gca,'ylim');

        mytitle = 'Position';
        mylegend = ' ';
        back_axes(mytitle,mylegend);
        figure(h)

        case 'velocity'
            h = figure();
            subplot(2,2,1)
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.store_velDes(1,:),'color',gray); hold on;
                plot(ODOM.odom_time, ODOM.vel_odom(1,:),'color',rgbk(1,:))     
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])
                title('x vel')
                ax_y = get(gca,'ylim');
            subplot(2,2,2)
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.store_velDes(2,:),'color',gray); hold on;
                plot(ODOM.odom_time, ODOM.vel_odom(2,:),'color',rgbk(2,:)) 
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])      
                title('y vel')
                ax_y = get(gca,'ylim');
            subplot(2,2,3)
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.store_velDes(3,:),'color',gray); hold on;
                plot(ODOM.odom_time, ODOM.vel_odom(3,:),'color',rgbk(3,:))   
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])  
                title('z vel')
                ax_y = get(gca,'ylim');
            subplot(2,2,4)
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.store_velDes(4,:),'color',gray); hold on;
                plot(ODOM.odom_time, ODOM.ang_odom(3,:),'color',rgbk(4,:))
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])      
                title('yaw vel')
                ax_y = get(gca,'ylim');
                
            mytitle = 'Velocity';
            mylegend = ' ';
            back_axes(mytitle,mylegend);
            figure(h)

        case 'acceleration'
            h = figure();
%             idx = ~isnan(ROBOT_HISTORY.time);
%             idx(1) = 0;
%             time = ROBOT_HISTORY.time(idx);
%             vel = ODOM.vel_odom(idx);
%             dv = vel(:,2:end) - vel(:,1:end-1);
%             Ot = ODOM.odom_headerT(idx);
%             dt = Ot(2:end) - Ot(1:end-1);
%             dvdt = dv./dt;
            titles = {'x accel','y accel','z accel'};
            for j = 1:3
            subplot(3,1,j)
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.store_accDes(j,:),'color',gray); hold on;
                plot(ODOM.odom_time, ODOM.acc_odom(j,:),'color',rgbk(j,:))  
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])
                title(titles{j});
            end     
            mytitle = 'Linear Acceleration';
            mylegend = ' ';
            back_axes(mytitle,mylegend);            
            figure(h)
            
        case 'position_error'
            h = figure();

            sa(1) = subplot(2,2,1);
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])
                title('error x')          
            sa(2) = subplot(2,2,2);
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])      
                title('error y')
            sa(3) = subplot(2,2,3);
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])  
                title('error z')
            sa(4) = subplot(2,2,4);
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])      
                title('error yaw')

            for j = 1:4
                axes(sa(j))
                hold on
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.pos_error(j,:),'color',rgbk(j,:));
                myx = get(gca,'xlim');
                myy = get(gca,'ylim');
                for i = 1:length(ROBOT_TRAJECTORY.timeline)
                    line([1 1]*(ROBOT_TRAJECTORY.timeline(i)),myy,'color',gray)
                end
                set(gca,'xlim',myx);
                set(gca,'ylim',myy);
            end
            
            mytitle = 'Position Error';
            mylegend = ' ';
            back_axes(mytitle,mylegend);            
            figure(h)
            
        case 'velocity_error'
            h=figure();
            sa(1) = subplot(2,2,1);
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.vel_error(1,:),'color',rgbk(1,:)); hold on;
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])
                title('velocity error x')          
            sa(2) = subplot(2,2,2);
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.vel_error(2,:),'color',rgbk(2,:)); hold on;
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])      
                title('velocity error y')
            sa(3) = subplot(2,2,3);
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.vel_error(3,:),'color',rgbk(3,:)); hold on;
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])  
                title('velocity error z')
            sa(4) = subplot(2,2,4);
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.vel_error(4,:),'color',rgbk(4,:)); hold on;
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])      
                title('angular velocity error yaw')

            for j = 1:4
                axes(sa(j))
                hold on
%                 plot(ROBOT_HISTORY.time, ROBOT_HISTORY.vel_error(j,:),'color',rgbk(j,:));
                myx = get(gca,'xlim');
                myy = get(gca,'ylim');
                for i = 1:length(ROBOT_TRAJECTORY.timeline)
                    line([1 1]*(ROBOT_TRAJECTORY.timeline(i)),myy,'color',gray)
                end
                set(gca,'xlim',myx);
                set(gca,'ylim',myy);
            end
            
            mytitle = 'Velocity Error';
            mylegend = ' ';
            back_axes(mytitle,mylegend);            
            figure(h)
            
        case 'cumulative_position_error'
           h = figure();
           ylabels = {'x error [m]','y error [m]','z error [m]'};
            for j = 1:3
                subplot(3,1,j)
                sorted_abs_error = sort(abs(ROBOT_HISTORY.pos_error(j,~isnan(ROBOT_HISTORY.pos_error(j,:)))));
                total_measurement = length(sorted_abs_error);
                measurement_frac = zeros(1,length(sorted_abs_error));
                for i = 1:total_measurement
                    measurement_frac(i) = sum(sorted_abs_error<sorted_abs_error(i))/total_measurement;
                end
                plot(measurement_frac, sorted_abs_error,'color',rgbk(j,:),'marker','.');
                ylabel(ylabels{j});
                if j == 3
                    xlabel('fraction of measurements below error')
                end
            end
            
            mytitle = 'Cumulative Absolute Position Error';
            mylegend = ' ';
            back_axes(mytitle,mylegend);            
            figure(h)
            
        case 'rpms'
            h = figure();
                plot(ROBOT_HISTORY.time, ROBOT_HISTORY.store_rpm'); hold on;
                legend('motor1','motor2','motor3', 'motor4')

%             pwm_min = 1000;           
%             pwm_max = 1950;        
%             pwm_rpm_scale = 16500;  
%             pwm_idle = 1080; 
%             pwm2rpm = @(pwm) (pwm - pwm_min).*pwm_rpm_scale ./ ( pwm_max - pwm_min );

                rpm_min = 3000;%pwm2rpm(pwm_min);
                rpm_max = 20000;%pwm2rpm(pwm_max);
                rpm_hover = 14000;%13600;
%                 rpm_idl = pwm2rpm(pwm_idle);

                ax_x = get(gca,'xlim'); ax_y = get(gca,'ylim');
                line(ax_x,[1 1]*rpm_min,'color',[1 1 1]*200/255);
                line(ax_x,[1 1]*rpm_max,'color',[1 1 1]*200/255);
%                 line(ax_x,[1 1]*rpm_idl,'color',[1 1 1]*200/255);
                line(ax_x,[1 1]*rpm_hover,'color',[1 1 1]*200/255);
                axis([ax_x rpm_min-200 rpm_max+200])
                
            title('rpms')
            figure(h);   

        case 'yaw_angular_rate'
            h =figure();
            plot(ROBOT_HISTORY.time,ROBOT_HISTORY.store_velDes(4,:),'k');
            hold on; plot(ODOM.odom_time,ODOM.ang_odom(3,:),'b');
            title('ang rate yaw desired')
            legend('desired angular rate','actual angular rate')
            figure(h)

        case 'attitude'
            global roll_step
            h = figure();
            ylabels = {'roll [rad]','pitch [rad]','yaw [rad]'};
            for p = 1:3  
                subplot(3,1,p)
                plot(ODOM.odom_time, ODOM.att_odom(p,:),'color',rgbk(p,:),'marker','.','linestyle','none'); hold on;
%                 plot(ODOM.odom_time, ODOM.att_odom_smoothed(p,:),'color',rgbk(4,:)); hold on;
                set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])
                myx = get(gca,'xlim');
                myy = get(gca,'ylim');
                rd = -roll_step;
                for i = 1:length(ROBOT_TRAJECTORY.timeline)
                    line([1 1]*(ROBOT_TRAJECTORY.timeline(i)),myy,'color',gray)
                    if strcmp(behavior,'step-in-roll')
                        if strcmp(ROBOT_TRAJECTORY.behavior{i},'roll__') && p==1
                            rd = -rd;
                            line([ROBOT_TRAJECTORY.timeline(i) ROBOT_TRAJECTORY.timeline(i+1)],rd*[1 1],'color',gray)
                        end
                    end
                end
                set(gca,'xlim',myx);
                set(gca,'ylim',myy);
                ylabel(ylabels{p});
                if p == 3
                    xlabel('time [s]')
                end
            end   
            mytitle = 'Attitude';
            mylegend = ' ';
            back_axes(mytitle,mylegend);            
            figure(h)       
        
        case 'xy_plane'
            figure()
            plot3(ODOM.pos_odom(1,:),ODOM.pos_odom(2,:),ODOM.odom_time)
            plot3(ROBOT_HISTORY.store_posDes(1,:),ROBOT_HISTORY.store_posDes(2,:),ROBOT_HISTORY.time,'r')
            xlabel('x')
            ylabel('y')
            view(-90,90)
            title('xy position')

        case 'xyzcon'
            h = figure();
                plot(ROBOT_HISTORY.time,ROBOT_HISTORY.store_xyzcon(1,:),'r'); hold on
                plot(ROBOT_HISTORY.time,ROBOT_HISTORY.store_xyzcon(2,:),'g'); 
                plot(ROBOT_HISTORY.time,ROBOT_HISTORY.store_xyzcon(3,:),'b'); 
                legend('xcon','ycon','zcon')
                line([1 1]*ROBOT_TRAJECTORY.timeline(1),get(gca,'ylim'),'color',gray)
            figure(h)

        case 'qdes'
            h = figure();
                plot(ROBOT_HISTORY.time,ROBOT_HISTORY.store_qdes); 
                title('qdes')
            figure(h)
        
        case 'omgd'
            h = figure();
                plot(ROBOT_HISTORY.time,ROBOT_HISTORY.store_omgd); 
                title('omg\_des, omg\_dot\_des')
                legend('phi\_dot','theta\_dot','psi\_dot','phi\_ddot','theta\_ddot','psi\_ddot')                
            figure(h)

        case 'moving average'
        for derivs = 1:2     
            
            idx =  ~isnan(ROBOT_HISTORY.pos_error(1,:));
            
        switch derivs
            case 1
                x = ROBOT_HISTORY.pos_error(1,idx);
                y = ROBOT_HISTORY.pos_error(2,idx);
                z = ROBOT_HISTORY.pos_error(3,idx);
                yaw = ROBOT_HISTORY.pos_error(4,idx);
                units = '';
            case 2
                x = ROBOT_HISTORY.vel_error(1,idx);
                y = ROBOT_HISTORY.vel_error(2,idx);
                z = ROBOT_HISTORY.vel_error(3,idx);
                yaw = ROBOT_HISTORY.vel_error(4,idx);  
                units = '/s';
        end
            
            time = ROBOT_HISTORY.time(idx);
            average_dt = mean( time(2:end)-time(1:end-1) );
            time_window = 1; % seconds
            window = round(time_window/average_dt);
            
            moving_avg = zeros(4,sum(idx)-2*window);
            moving_std = zeros(4,sum(idx)-2*window);
            count = 1;
            for i = window:sum(idx)-window
                moving_avg(:,count) = mean( [x(count:i); ...
                                             y(count:i); ...
                                             z(count:i); ...
                                             yaw(count:i) ],2 );
                moving_std(:,count) = std( [x(count:i); ...
                                             y(count:i); ...
                                             z(count:i); ...
                                             yaw(count:i) ],1,2 );                                     
                count = count+1;
            end
        
            h = figure();
                T = ROBOT_HISTORY.time(window:sum(idx)-window); 
                
            h1 = subplot(2,2,1);
            h1pos = get(h1,'position');
            h1pos(2) = h1pos(2)-.05;
            set(h1,'position',h1pos);            
                Xstd = moving_std(1,:);
                Xmean = moving_avg(1,:);
                polyV = [T fliplr(T); Xmean+Xstd fliplr(Xmean-Xstd)];          
                p = patch(polyV(1,:),polyV(2,:),-ones(1,length(polyV)));
                set(p,'FaceColor',pastel_red)
                set(p,'EdgeColor',pastel_black); hold on
                plot(T, moving_avg(1,:),'color',rgbk(1,:));    
                set(gca,'xlim',[0 ROBOT_HISTORY.time(sum(idx))])
                ylabel(['x error [m' units ']']); xlabel('time [s]')

            h2 = subplot(2,2,2);
            h2pos = get(h2,'position');
            h2pos(2) = h2pos(2)-.05;
            set(h2,'position',h2pos);                
                Xstd = moving_std(2,:);
                Xmean = moving_avg(2,:);
                polyV = [T fliplr(T); Xmean+Xstd fliplr(Xmean-Xstd)];          
                p = patch(polyV(1,:),polyV(2,:),-ones(1,length(polyV)));
                set(p,'FaceColor',pastel_green)
                set(p,'EdgeColor',pastel_black); hold on
                plot(T, moving_avg(2,:),'color',rgbk(2,:));    
                set(gca,'xlim',[0 ROBOT_HISTORY.time(sum(idx))])
                ylabel(['y error [m' units ']']); xlabel('time [s]')
 
            subplot(2,2,3)
                Xstd = moving_std(3,:);
                Xmean = moving_avg(3,:);
                polyV = [T fliplr(T); Xmean+Xstd fliplr(Xmean-Xstd)];          
                p = patch(polyV(1,:),polyV(2,:),-ones(1,length(polyV)));
                set(p,'FaceColor',pastel_blue)
                set(p,'EdgeColor',pastel_black); hold on
                plot(T, moving_avg(3,:),'color',rgbk(3,:));    
                set(gca,'xlim',[0 ROBOT_HISTORY.time(sum(idx))])
                ylabel(['z error [m' units ']']); xlabel('time [s]')

            subplot(2,2,4)
                Xstd = moving_std(4,:);
                Xmean = moving_avg(4,:);
                polyV = [T fliplr(T); Xmean+Xstd fliplr(Xmean-Xstd)];          
                p = patch(polyV(1,:),polyV(2,:),-ones(1,length(polyV)));
                set(p,'FaceColor',pastel_black)
                set(p,'EdgeColor',gray); hold on
                plot(T, moving_avg(4,:),'color',rgbk(4,:));    
                set(gca,'xlim',[0 ROBOT_HISTORY.time(sum(idx))])
                ylabel(['yaw error [rad' units ']']); xlabel('time [s]')
            
            g = axes();
                plot(0, 0,'color','k'); hold on; 
                plot(0, 0,'color',pastel_black,'linewidth',10)  
            set(g,'Visible','off')
            switch derivs
                case 1
            title({behavior;['Position Error: Moving Average over a ' num2str(time_window) ' second window']},'fontsize',10,'fontweight','bold')
                case 2
            title({behavior;['Velocity Error: Moving Average over a ' num2str(time_window) ' second window']},'fontsize',10,'fontweight','bold')
            end
            l = legend('mean','std dev','location','SouthOutside','orientation','horizontal');
            lpos = get(l,'position'); lpos(2) = 0.005; set(l,'position',lpos)
            set(findall(gca, 'type', 'text'), 'visible', 'on')
            uistack(gca, 'bottom')
            drawnow
            figure(h)
        end

        case 'RMS'

        for derivs = 1:2     
            
            idx =  ~isnan(ROBOT_HISTORY.time(1,:));
            T = ROBOT_HISTORY.time(idx); 
            
        switch derivs
            case 1
                myRMS = rms(ROBOT_HISTORY.pos_error(1:3,idx));
                x = ROBOT_HISTORY.pos_error(1,idx);
                y = ROBOT_HISTORY.pos_error(2,idx);
                z = ROBOT_HISTORY.pos_error(3,idx);
                yaw = ROBOT_HISTORY.pos_error(4,idx);
                units = '';
            case 2
                myRMS = rms(ROBOT_HISTORY.vel_error(1:3,idx));
                x = ROBOT_HISTORY.vel_error(1,idx);
                y = ROBOT_HISTORY.vel_error(2,idx);
                z = ROBOT_HISTORY.vel_error(3,idx);
                yaw = ROBOT_HISTORY.vel_error(4,idx);  
                units = '/s';
        end
                   
            h = figure();
            h1 = axes;
                h1pos = get(h1,'position');
                h1pos(2) = h1pos(2)-.01;
                h1pos(3) = .7*h1pos(3);
                set(h1,'position',h1pos);            
                
                plot(T, myRMS,'k');    
                set(gca,'xlim',[0 ROBOT_HISTORY.time(sum(idx))])
                ylabel(['RMS error [m' units ']']); xlabel('time [s]')
            
            h2 = axes;
                h2pos = get(h2,'position');
                endlength = h2pos(1)+h2pos(3);
                h2pos(2) = h1pos(2);
                h2pos(1) = h1pos(1)+h1pos(3)+.025;
                h2pos(3) = endlength - h2pos(1);
                set(h2,'position',h2pos); 
                
                text(0,.9,'Error Stats','VerticalAlignment','top')
                
                text(0,.8,['mean error X: ' num2str(mean(x)) ' m' units],'VerticalAlignment','top')
                if mean(x)<0
                    space = '   ';
                else
                    space = '  ';
                end
                text(0,.75,['max error X: ' space num2str(max(abs(x))) ' m' units],'VerticalAlignment','top')

                text(0,.6,['mean error Y: ' num2str(mean(y)) ' m' units],'VerticalAlignment','top')
                if mean(y)<0
                    space = '   ';
                else
                    space = '  ';
                end
                text(0,.55,['max error Y: ' space num2str(max(abs(y))) ' m' units],'VerticalAlignment','top')

                text(0,.4,['mean error Z: ' num2str(mean(z)) ' m' units],'VerticalAlignment','top')
                if mean(z)<0
                    space = '   ';
                else
                    space = '  ';
                end
                text(0,.35,['max error Z: ' space num2str(max(abs(z))) ' m' units],'VerticalAlignment','top')
                
                text(0,.2,['mean error yaw: ' num2str(mean(yaw)) ' rad' units],'VerticalAlignment','top')
                if mean(yaw)<0
                    space = '   ';
                else
                    space = '  ';
                end
                text(0,.15,['max error yaw: ' space num2str(max(abs(yaw))) ' rad' units],'VerticalAlignment','top')

                set(h2,'Visible','off')

            g = axes();
                plot(0, 0,'color','k'); hold on; 
                plot(0, 0,'color',pastel_black,'linewidth',10)  
            set(g,'Visible','off')
            switch derivs
                case 1
            title({behavior;'RMS Position Error'},'fontsize',10,'fontweight','bold')
                case 2
            title({behavior;'RMS Velocity Error'},'fontsize',10,'fontweight','bold')
            end
            set(findall(gca, 'type', 'text'), 'visible', 'on')
            uistack(gca, 'bottom')
            drawnow
            figure(h)
        end       
    
        case 'histogram'
    
        for derivs = 1:2     
            
            idx =  ~isnan(ROBOT_HISTORY.time(1,:));
            T = ROBOT_HISTORY.time(idx); 
            
        switch derivs
            case 1
                x = ROBOT_HISTORY.pos_error(1,idx);
                y = ROBOT_HISTORY.pos_error(2,idx);
                z = ROBOT_HISTORY.pos_error(3,idx);
                yaw = ROBOT_HISTORY.pos_error(4,idx);
                units = '';
            case 2
                x = ROBOT_HISTORY.vel_error(1,idx);
                y = ROBOT_HISTORY.vel_error(2,idx);
                z = ROBOT_HISTORY.vel_error(3,idx);
                yaw = ROBOT_HISTORY.vel_error(4,idx);  
                units = '/s';
        end
                   
            h = figure();
                down = .025;
                nbins = 50;
               
            h1 = subplot(2,2,1);
            h1pos = get(h1,'position');
            h1pos(2) = h1pos(2)-down;
            set(h1,'position',h1pos);     
                hist(h1,x,nbins)
                p = findobj(gca,'Type','patch');
                set(p,'FaceColor',rgbk(1,:));
%                 set(p,'EdgeColor','w'); 
                xlabel(['x error [m' units ']']); ylabel('count')

            h2 = subplot(2,2,2);
            h2pos = get(h2,'position');
            h2pos(2) = h2pos(2)-down;
            set(h2,'position',h2pos);            
                hist(h2,y,nbins)
                p = findobj(gca,'Type','patch');
                set(p,'FaceColor',rgbk(2,:));
%                 set(p,'EdgeColor','w'); 
                xlabel(['y error [m' units ']']); ylabel('count')
                
            h3 = subplot(2,2,3);
            h3pos = get(h3,'position');
            h3pos(2) = h3pos(2)-down;
            set(h3,'position',h3pos);              
                hist(h3,z,nbins)
                p = findobj(gca,'Type','patch');
                set(p,'FaceColor',rgbk(3,:));
%                 set(p,'EdgeColor','w'); 
                xlabel(['z error [m' units ']']); ylabel('count')

            h4 = subplot(2,2,4);
            h4pos = get(h4,'position');
            h4pos(2) = h4pos(2)-down;
            set(h4,'position',h4pos);              
                hist(h4,yaw,nbins)
                p = findobj(gca,'Type','patch');
                set(p,'FaceColor',rgbk(4,:));
%                 set(p,'EdgeColor','w'); 
                xlabel(['yaw error [rad' units ']']); ylabel('count')
            
            g = axes();
            set(g,'Visible','off')
            switch derivs
                case 1
            title({behavior;['Histogram of Position Error']},'fontsize',10,'fontweight','bold')
                case 2
            title({behavior;['Histogram of Velocity Error']},'fontsize',10,'fontweight','bold')
            end
            set(findall(gca, 'type', 'text'), 'visible', 'on')
            uistack(gca, 'bottom')
            drawnow
            figure(h)
        end
    
        case 'fft'
    try
        for derivs = 1:2     
            
            idx =  ~isnan(ROBOT_HISTORY.time(1,:));
            T = ROBOT_HISTORY.time(idx); 
            
        switch derivs
            case 1
                x = ROBOT_HISTORY.pos_error(1,idx);
                y = ROBOT_HISTORY.pos_error(2,idx);
                z = ROBOT_HISTORY.pos_error(3,idx);
                yaw = ROBOT_HISTORY.pos_error(4,idx);
                units = '';
            case 2
                x = ROBOT_HISTORY.vel_error(1,idx);
                y = ROBOT_HISTORY.vel_error(2,idx);
                z = ROBOT_HISTORY.vel_error(3,idx);
                yaw = ROBOT_HISTORY.vel_error(4,idx);  
                units = '/s';
        end
                   
            h = figure();

            h1 = subplot(2,2,1);
            h1pos = get(h1,'position');
            h1pos(2) = h1pos(2)-down;
            set(h1,'position',h1pos);     
                stem((fft(x)),'color',rgbk(1,:))
                xlabel(['frequency x error [m' units ']']); ylabel('magnitude')

            h2 = subplot(2,2,2);
            h2pos = get(h2,'position');
            h2pos(2) = h2pos(2)-down;
            set(h2,'position',h2pos);            
                stem((fft(y)),'color',rgbk(2,:))
                xlabel(['frequency y error [m' units ']']); ylabel('magnitude')
                
            h3 = subplot(2,2,3);
            h3pos = get(h3,'position');
            h3pos(2) = h3pos(2)-down;
            set(h3,'position',h3pos);              
                stem((fft(z)))
                xlabel(['frequency z error [m' units ']']); ylabel('magnitude')

            h4 = subplot(2,2,4);
            h4pos = get(h4,'position');
            h4pos(2) = h4pos(2)-down;
            set(h4,'position',h4pos);              
                stem((fft(yaw)),'color','k')
                xlabel(['frequency yaw error [m' units ']']); ylabel('magnitude')
            
            g = axes();
            set(g,'Visible','off')
            switch derivs
                case 1
            title({behavior;['FFT of Position Error']},'fontsize',10,'fontweight','bold')
                case 2
            title({behavior;['FFT of Velocity Error']},'fontsize',10,'fontweight','bold')
            end
            set(findall(gca, 'type', 'text'), 'visible', 'on')
            uistack(gca, 'bottom')
            drawnow
            figure(h)
        end
    catch
        display('fft plot throws error')
    end
        
        case 'IMU_vs_Vicon'
    try
        orange = [255 165 0]/255;
        green = [34 139 34]/255;
        teal = [32 178 170]/255;
    h = figure();
    subplot(1,2,1)
        plot(ODOM.odom_time, ODOM.att_odom(1,:),'r','linewidth',2); hold on;
        plot(IMU.imu_time, IMU.att_imu(1,:),'color',orange,'linewidth',2);
        set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])
        title('roll'); xlabel('time [s]'); ylabel('rad');
        legend('Vicon','IMU','location','best')
    subplot(1,2,2)
        plot(ODOM.odom_time, ODOM.att_odom(2,:),'color',green,'linewidth',2); hold on;
        plot(IMU.imu_time, IMU.att_imu(2,:),'color',teal,'linewidth',2);
        set(gca,'xlim',[0 ROBOT_TRAJECTORY.timeline(end)])
        title('pitch'); xlabel('time [s]'); ylabel('rad');
        legend('Vicon','IMU','location','best')
    catch
        display('IMU_vs_Vicon plot throws error')
    end
    
        case 'XYZ_plot3'
            h = figure();
            data = ROBOT_HISTORY.store_posDes;
            h1 = plot3( data(1,:),data(2,:),data(3,:),'color','k' ); 
            hold on;             
            
            % color the trajectory over time
                cdata = colormap(jet);
                data = ODOM.pos_odom;
                idx = ~isnan(data(1,:));
                data = data(:,idx);
                time = ROBOT_HISTORY.time(~isnan(ROBOT_HISTORY.time));
                dcol = round(size(data,2)/64);
            for seg=1:64
                foo = dcol*(seg-1)+1;
                bar = foo+dcol-1;
                if bar > size(data,2)
                    bar = size(data,2);
                end
                
                h2 = plot3( data(1,foo:bar),data(2,foo:bar),data(3,foo:bar),...
                       'color',cdata(seg,:),'marker','.' ); 
            end   
            if bar<size(data,2)
                foo = bar;
                bar = size(data,2);
                plot3( data(1,foo:bar),data(2,foo:bar),data(3,foo:bar),...
                       'color',cdata(end,:) ); 
            end
            grid on; view(0,90); axis equal; colormap(jet); 
            g = colorbar;
            g.Label.String = 'time [s]';
            g.Label.Position = [-1 0.5 0];
            g.Ticks = [ 0 .5 1 ];
            g.TickLabels = {[num2str(time(1)) 's'], ...
                            [num2str(time(floor(length(time)/2))) 's'], ...
                            [num2str(time(end)) 's']};
            title('Spatial'); xlabel({'X [m]';' '}); ylabel('Y [m]'); zlabel('Z [m]')
            legend([h1,h2],'desired','actual','location','southOutside','orientation','horizontal')   
            view(30,15)
            axis square
    
        otherwise
            display(['sorry, no plots for string "' name '"'])
    end

end

end

function [] = back_axes(mytitle,mylegend)
    g = axes();
    set(g,'Visible','off')
    title(mytitle,'fontsize',10,'fontweight','bold')
    set(findall(gca, 'type', 'text'), 'visible', 'on')
    uistack(gca, 'bottom')
    drawnow
end
