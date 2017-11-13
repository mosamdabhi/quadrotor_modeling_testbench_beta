function F = ViconModel(t, W, omega, accel)

    % Quaternion derivative
    capitalOmega = [-skewmat(omega), omega;
                        -omega',        0];
    F(1:4,1) = 0.5.*(capitalOmega*W(1:4));
    
    F(5:7,1) = W(8:10);
    F(8:10,1) = accel(1:3)+[0;0;9.80665];
end