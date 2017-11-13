function qFinal = InverseQuat(q)
 if isstruct(q)
    qFinal(1) = -1*q.x;
    qFinal(2) = -1*q.y;
    qFinal(3) = -1*q.z;
    qFinal(4) = q.w;
 else
    qFinal(1) = -q(1);
    qFinal(2) = -q(2);
    qFinal(3) = -q(3);
    qFinal(4) = q(4);
 end
end