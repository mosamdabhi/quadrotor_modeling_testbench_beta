function S = skew(W)

w1 = W(1);
w2 = W(2);
w3 = W(3);

S = [  0  -w3   w2; ...
      w3    0  -w1; ...
     -w2   w1    0];

end