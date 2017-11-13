function z = NonlinearMeasurementModel(X)
%     qIGEst = quat_struct(X(1:4,1));
%     qIVEst = quat_struct(X(11:14,1));
    z(1:4,1) = MultiplyQuat(X(1:4,1), (InverseQuat(X(11:14,1))), 1);
    qVGEst = quat_struct(z(1:4,1));
    z(5:7,1) = X(8:10,1) - (QuatToR(qVGEst))'*X(15:17,1);
end
