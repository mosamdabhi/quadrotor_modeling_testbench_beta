function R = ZYXToR(eulerAngles)
phi = eulerAngles(1);
theta = eulerAngles(2);
psi = eulerAngles(3);

R = zeros(3, 3);

R(1, 1) = cos(theta)*cos(psi);
R(1, 2) = cos(psi)*sin(theta)*sin(phi) - cos(phi)*sin(psi);
R(1, 3) = cos(phi)*cos(psi)*sin(theta) + sin(phi)*sin(psi);

R(2, 1) = cos(theta)*sin(psi);
R(2, 2) = cos(phi)*cos(psi) + sin(theta)*sin(phi)*sin(psi);
R(2, 3) = -cos(psi)*sin(phi) + cos(phi)*sin(theta)*sin(psi);

R(3, 1) = -sin(theta);
R(3, 2) = cos(theta)*sin(phi);
R(3, 3) = cos(theta)*cos(phi);
