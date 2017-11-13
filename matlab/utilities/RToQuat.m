function q = RToQuat(R)

[v, d] = eig(R - eye(3));

d = diag(abs(d));
[s, ind] = sort(d);
if d(ind(1)) > 0.001
    warning('Rotation matrix is dubious');
end

ax = v(:,ind(1));

if abs(norm(ax) - 1) > .0001
    warning('non unit rotation axis');
end

twocostheta = trace(R) - 1;
twosinthetav = [R(3,2) - R(2,3), ...
                R(1,3) - R(3,1), ...
                R(2,1) - R(1,2)]';
twosintheta = ax'*twosinthetav;

theta = atan2(twosintheta, twocostheta);

quat = [ax*sin(theta/2); cos(theta/2)];

q.x = quat(1);
q.y = quat(2);
q.z = quat(3);
q.w = quat(4);
