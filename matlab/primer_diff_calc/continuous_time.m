function F = continuous_time(t, augW, a, omega, QuadModel)
    % Quaternion derivative
    q.x = augW(7);
    q.y = augW(8);
    q.z = augW(9);
    q.w = augW(10);
    R = QuatToR(q);
    F(1:3,1) = augW(4:6,1);
    F(4:6,1) = [0;0;QuadModel.gravity] + R'*a;% - skewmat(omega)*augW(4:6,1);
    capitalOmega = [-skewmat(omega), omega;
                        -omega',        0];
    F(7:10,1) = 0.5.*(capitalOmega*augW(7:10));   
end
