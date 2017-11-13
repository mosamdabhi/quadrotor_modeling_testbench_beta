function matrixex = distrans(F, F21, dt)
m = F.*dt;
m(4:6, 1:3) = F21;

matrixex = expm(m);
end