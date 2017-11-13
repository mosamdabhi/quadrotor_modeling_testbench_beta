function [pos_des,vel_des,acc_des,jrk_des] = poly5(t,tf,start_vals,end_vals)

% start_vals & end_vals have the form: 
%    
%               1st col = pos,   2nd col = vel,   3rd col = acc
% 1st row = x
% 2nd row = y
% 3rd row = z
% 4th row = psi

% if we didn't input vel or acc, assume zeros
if size(start_vals,2)==1
    start_vals = [start_vals zeros(size(start_vals,1),2)];
end
if size(start_vals,2)==2
    start_vals = [start_vals zeros(size(start_vals,1),1)];
end    
if size(end_vals,2)==1
    end_vals = [end_vals zeros(size(end_vals,1),2)];
end
if size(end_vals,2)==2
    end_vals = [end_vals zeros(size(end_vals,1),1)];
end   

% tf = 2.4;  t = 0:.1:tf; 

rows = size(start_vals,1);

pos_des = zeros(rows,length(t));
vel_des = zeros(rows,length(t));
acc_des = zeros(rows,length(t));
jrk_des = zeros(rows,length(t));

for xyz = 1:rows
      
    xf = end_vals(xyz,1);    
    vf = end_vals(xyz,2);
    af = end_vals(xyz,3);
    
    f = start_vals(xyz,1);
    e = start_vals(xyz,2);
    d = start_vals(xyz,3)/2;
    
    a = -(12*f - 12*xf + 6*e*tf + 6*tf*vf - af*tf^2 + 2*d*tf^2)/(2*tf^5);
    b = (15*f - 15*xf + 8*e*tf + 7*tf*vf - af*tf^2 + 3*d*tf^2)/tf^4;
    c = -(20*f - 20*xf + 12*e*tf + 8*tf*vf - af*tf^2 + 6*d*tf^2)/(2*tf^3);

    pos  =    a*t.^5 +    b*t.^4 +   c*t.^3 +   d*t.^2 + e*t + f;
    vel  =  5*a*t.^4 +  4*b*t.^3 + 3*c*t.^2 + 2*d*t   + e;
    acc  = 20*a*t.^3 + 12*b*t.^2 + 6*c*t   + 2*d;
    jrk  = 60*a*t.^2 + 24*b*t   + 6*c;

    pos_des(xyz,:) = pos;
    vel_des(xyz,:) = vel;
    acc_des(xyz,:) = acc;
    jrk_des(xyz,:) = jrk;
end

%{        
figure()
set(gca,'ColorOrder',yaycolors); hold on;
plot(t,pos_des)

figure()
set(gca,'ColorOrder',yaycolors); hold on;
plot(t,vel_des)

figure()
set(gca,'ColorOrder',yaycolors); hold on;
plot(t,acc_des)

figure()
set(gca,'ColorOrder',yaycolors); hold on;
plot(t,jrk_des)
%}
end