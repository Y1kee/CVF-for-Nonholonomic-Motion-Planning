function T = source(p)
%% Source VF centering at origin with unit magnitude
x = p(1); y = p(2);
r = norm(p);
Tx = x/r;
Ty = y/r;
T = [Tx;Ty];

end