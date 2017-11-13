function errstate = errstate_compute(W1, W2)
    A = MultiplyQuat(W1(1:4,1), InverseQuat(W2(1:4,1)),1);
    B = MultiplyQuat(W1(11:14,1), InverseQuat(W2(11:14,1)),1);
    errstate(1:3,1) = 2.*A(1:3, 1);
    errstate(4:6,1) = W1(5:7,1) - W2(5:7, 1);
    errstate(7:9,1) = W1(8:10,1) - W2(8:10, 1);
    errstate(10:12,1) = 2.*B(1:3, 1);
    errstate(13:15,1) = W1(15:17,1) - W2(15:17, 1);
end