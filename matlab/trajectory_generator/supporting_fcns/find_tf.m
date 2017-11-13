function [tf] = find_tf(dx,vdes,aabs)
%% 
%{
%we assume a move will only take max 5min, and we check the accel in 
% increments of .1 from 1s to 2min & return the first time that returns
% accels under aabs
% 
% amax = aabs;
% amin = -aabs;
% 
% t = 1:.02:60;
% 
% t4 = t.^4;
% t3 = t.^3;
% t2 = t.^2;
% t1 = t;
% 
% a3 = -120*dx;
% a2 = 180*dx;
% a1 = -60*dx;
% 
% v4 = -30*dx;
% v3 = 60*dx;
% v2 = -30*dx;
% 
% tf5 = t.^5;
% tf4 = t.^4;
% tf3 = t.^3;
% 
% Tn4 = repmat(t4',1,length(t));
% Tn3 = repmat(t3',1,length(t));
% Tn2 = repmat(t2',1,length(t));
% Tn1 = repmat(t1',1,length(t));
% 
% Tf5 = repmat(tf5,length(t),1);
% Tf4 = repmat(tf4,length(t),1);
% Tf3 = repmat(tf3,length(t),1);
% 
% acc = a3*Tn3./Tf5 + a2*Tn2./Tf4 + a1*Tn1./Tf3;
% vel = v4*Tn4./Tf5 + v3*Tn3./Tf4 + v2*Tn2./Tf3;
% 
% acceptable_ACC = sum(triu(acc<=amax & acc>=amin),1);
% acceptable_ACC = acceptable_ACC==1:length(t);
% offset = find(acceptable_ACC-1,1);
% idx = find(acceptable_ACC(offset:end),1);
% if isempty(idx)
%     tf_a = t(1);
% else
%     tf_a = t(idx);  
% end
% 
% acceptable_VEL = sum(triu(vel<=vdes & vel>=-vdes),1);
% acceptable_VEL = acceptable_VEL==1:length(t);
% offset = find(acceptable_VEL-1,1);
% idx = find(acceptable_VEL(offset:end),1);
% tf_v = t(offset+idx-1);
% if isempty(tf_v)
%     tf = tf_a;
% else
%     tf = max(tf_a,tf_v);
% end
%}
%% much faster; only ~valid when 5th order poly & vel&acc start & end @ 0
test_af = @(dx,ma) .5*sqrt(-4/ma*(-6*20*dx/4^3 + 12*15*dx/4^2 + 6*-10*dx/4));
tf_a = test_af(abs(dx),abs(aabs));

test_vf = @(dx,mv) 15/8*dx/mv;
tf_v = test_vf(abs(dx),abs(vdes));

% safety, just for fun, & we never return less than 1s, again, for safety
safety = .05;
tf = max([tf_a+safety,tf_v+safety,1]);

end