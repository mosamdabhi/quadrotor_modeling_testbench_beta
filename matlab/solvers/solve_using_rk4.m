function [W_final, lin_ang_accel_save] = solve_using_rk4(t, W, QuadModel, desRPM, h)
    K = zeros(length(W), 4);
    K(:,1) = QuadrotorDynamics(t, W, QuadModel, desRPM);
    K(:,2) = QuadrotorDynamics(t + 0.5*h, W + 0.5*h*K(:,1), QuadModel, desRPM);
    K(:,3) = QuadrotorDynamics(t + 0.5*h, W + 0.5*h*K(:,2), QuadModel, desRPM);
    K(:,4) = QuadrotorDynamics(t + h, W + h*K(:,3), QuadModel, desRPM);
    
    lin_ang_accel_save = K(1:6,1);
    lin_ang_accel_save(3,1) = lin_ang_accel_save(3,1) - QuadModel.gravity;
    
    W_final = W + (1/6)*(K(:,1) + 2*K(:,2) + 2*K(:,3) + K(:,4))*h;
end