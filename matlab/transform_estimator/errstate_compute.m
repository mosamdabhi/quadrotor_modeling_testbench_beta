function errstate = errstate_compute(W1, W2)
    A = MultiplyQuat(W1(7:10,1), InverseQuat(W2(7:10,1)),1);
    errstate(7:9,1) = 2.*A(1:3, 1);
    errstate(1:3,1) = W1(1:3,1) - W2(1:3, 1);
    errstate(4:6,1) = W1(4:6,1) - W2(4:6, 1);
    errstate(10:12,1) = W1(11:13,1) - W2(11:13, 1);
    errstate(13:15,1) = W1(14:16,1) - W2(14:16, 1);
    errstate(16:18,1) = W1(17:19,1) - W2(17:19, 1);
    errstate(19:22,1) = W1(20:23,1) - W2(20:23, 1);
end