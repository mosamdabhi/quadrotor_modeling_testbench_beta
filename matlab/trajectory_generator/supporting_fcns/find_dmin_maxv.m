function [tf, dmin] = find_dmin_maxv(vmax,amax)
%% estimate for 5th order poly w/ vel&acc start & end @ 0
amax = abs(amax);
vmax = abs(vmax);

dx = 0:.01:5;

% given dx and a max accel@t/4, it will take tf time to cover that distance 
test_af = @(dx,ma) .5*sqrt(-4/ma*(-6*20*dx/4^3 + 12*15*dx/4^2 + 6*-10*dx/4));
tf_a = test_af(dx,amax);

% given dx and a max vel@t/2, it will take tf time to cover that distance 
test_vf = @(dx,mv) 15/8*dx/mv;
tf_v = test_vf(dx,vmax);

% for said dx distance, which time was greater? that time is the min time
% we must never go below, to cover dx without exceeding a bound
tf = max(tf_a,tf_v);

% these vectors record which dereivitve mandated tf, to remain in-bounds
a_limit = bsxfun(@eq, tf_a, tf);
v_limit = ~a_limit;

% so, the shortest distance we can travel to hit the max velocity = 
dv = dx(v_limit);
dmin = dv(1)/2;

% then dt to go from vmax,a0 to dDv/2,v0,a0 should be the time limited by
% accel, but all the same, check again.
tf_a = test_af(dmin,amax);
tf_v = test_vf(dmin,vmax);
tf = max(tf_a,tf_v);

% visual confirmation
%{
t = 0:.01:tf;

xf = dmin;    
vf = 0;
af = 0;

f = 0;
e = vmax;
d = 0;

a = -(12*f - 12*xf + 6*e*tf + 6*tf*vf - af*tf.^2 + 2*d*tf.^2)./(2*tf.^5);
b = (15*f - 15*xf + 8*e*tf + 7*tf*vf - af*tf.^2 + 3*d*tf.^2)./tf.^4;
c = -(20*f - 20*xf + 12*e*tf + 8*tf*vf - af*tf.^2 + 6*d*tf.^2)./(2*tf.^3);

pos  =    a*t.^5 +    b*t.^4 +   c*t.^3 +   d*t.^2 + e*t + f;
vel  =  5*a*t.^4 +  4*b*t.^3 + 3*c*t.^2 + 2*d*t   + e;
acc  = 20*a*t.^3 + 12*b*t.^2 + 6*c*t   + 2*d;
jrk  = 60*a*t.^2 + 24*b*t   + 6*c;

   
yaycolors = [ 255 0 0; 34 139 34; 0 0 255 ]/255;     
figure()
set(gca,'ColorOrder',yaycolors); hold on;
plot(t,pos)

figure()
set(gca,'ColorOrder',yaycolors); hold on;
plot(t,vel)

figure()
set(gca,'ColorOrder',yaycolors); hold on;
plot(t,acc)

figure()
set(gca,'ColorOrder',yaycolors); hold on;
plot(t,jrk)
%}

end