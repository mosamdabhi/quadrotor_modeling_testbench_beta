function euler_ = QuatToZYX(q_)
    q_ = quat_struct(q_);
    R = QuatToR(q_);
    euler_ = RToZYX(R);
end