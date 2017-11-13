function [pos_xy, vel_xy, acc_xy, jrk_xy ] = circle_xy(myt, tf, xy_center, r)

% tf = 1; myt = 0:.01:tf;
T = 2*pi/tf;

pos_xy = zeros(2,length(myt));
vel_xy = zeros(2,length(myt));
acc_xy = zeros(2,length(myt));
jrk_xy = zeros(2,length(myt));


pos_xy(1:2,:) = repmat(xy_center,1,length(myt)) + r*[ sin(T*myt);  cos(T*myt)];
vel_xy(1:2,:) = T*r*[ cos(T*myt); -sin(T*myt) ];
acc_xy(1:2,:) = T^2*r*[ -sin(T*myt); -cos(T*myt) ];
jrk_xy(1:2,:) = T^3*r*[ -cos(T*myt); sin(T*myt) ];


end