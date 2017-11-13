function F = ProcessModel(t, augW, a, omega, QuadModel)
    % Quaternion derivative
    capitalOmega = [-skewmat(omega), omega;
                        -omega',        0];
    F(1:4,1) = 0.5.*(capitalOmega*augW(1:4));
    q.x = augW(1);
    q.y = augW(2);
    q.z = augW(3);
    q.w = augW(4);
    F(5:7,1) = QuatToR(q)'*a + [0; 0; QuadModel.gravity];
    F(8:10,1) = augW(5:7);
    F(11:14,1) = zeros(4,1);
    F(15:17,1) = zeros(3,1);
    
    deriv = -QuatToR(quat_struct(augW(1:4)))'*skewmat(a);
    F(18:26, 1) = reshape(deriv, [9,1]);
end
