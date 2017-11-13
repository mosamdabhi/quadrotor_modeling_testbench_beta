function W_ = propagate_state(W, E)
    % Error state E - 15x1 vector, State W - 17x1 vector
    % Output state W_ - 17x1 vector
    
    % Error quaternion normalization
    if (0.5.*E(1:3,1)')*(0.5.*E(1:3,1)) > 1
        delqIG = (1/sqrt(1 + (0.5.*E(1:3,1)')*(0.5.*E(1:3,1)))).*[0.5.*E(1:3,1); 1];
    else
        delqIG = [0.5.*E(1:3,1); sqrt(1 - (0.5.*E(1:3,1)')*(0.5.*E(1:3,1)))];
    end
    
    
    if (0.5.*E(10:12,1)')*(0.5.*E(10:12,1)) > 1
        delqIV = (1/sqrt(1 + (0.5.*E(10:12,1)')*(0.5.*E(10:12,1)))).*[0.5.*E(10:12,1); 1];
    else
        delqIV = [0.5.*E(10:12,1); sqrt(1 - (0.5.*E(10:12,1)')*(0.5.*E(10:12,1)))];
    end
    
%     delqIG = [0.5.*E(1:3,1); 1];
%     delqIV = [0.5.*E(10:12,1); 1];
    W_(1:4, 1) = MultiplyQuat(delqIG, (W(1:4, 1)), 1);
    W_(5:7, 1) = W(5:7, 1) + E(4:6, 1);
    W_(8:10, 1) = W(8:10, 1) + E(7:9, 1);
    W_(11:14, 1) = MultiplyQuat(delqIV, (W(11:14, 1)), 1);
    W_(15:17, 1) = W(15:17,1) + E(13:15, 1);

end
