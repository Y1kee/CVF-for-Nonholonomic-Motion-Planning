function T = vortex(p)
%% Spiral VF centering at origin with unit magnitude
x = p(1); y = p(2);
r = norm(p);
Tx = -y/r;
Ty = x/r;
T = [Tx;Ty];

end