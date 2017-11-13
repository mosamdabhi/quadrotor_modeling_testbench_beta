function qFinal = MultiplyQuat(q1, q2, i)
    if isstruct(q1)
    LHcompound = [q1.w(i) q1.z(i) -q1.y(i) q1.x(i);
                  -q1.z(i) q1.w(i) q1.x(i) q1.y(i);
                 q1.y(i) -q1.x(i) q1.w(i) q1.z(i);
                 -q1.x(i) -q1.y(i) -q1.z(i) q1.w(i)];
    qFinal = LHcompound*[q2(1); q2(2); q2(3); q2(4)];
    else
    LHcompound = [q1(4) q1(3) -q1(2) q1(1);
                  -q1(3) q1(4) q1(1) q1(2);
                 q1(2) -q1(1) q1(4) q1(3);
                 -q1(1) -q1(2) -q1(3) q1(4)];
    qFinal = LHcompound*[q2(1); q2(2); q2(3); q2(4)];
    end
end