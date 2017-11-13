function euler_zyx = RToZYX(R)

euler_zyx = zeros(3, 1);

theta = wrapToPi(-real(asin(R(3,1))));
if(abs(cos(theta)) > 1e-6)
    phi = wrapToPi(atan2(R(3,2), R(3,3)));
    psi = wrapToPi(atan2(R(2,1), R(1,1)));
else
    phi = 0;
    psi = 0;
end

euler_zyx(1) = phi;
euler_zyx(2) = theta;
euler_zyx(3) = psi;