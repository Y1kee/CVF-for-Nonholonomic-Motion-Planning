function T = sink(p)
%% Sink VF centering at origin with unit magnitude
x = p(1); y = p(2);
r = norm(p);
Tx = -x/r;
Ty = -y/r;
T = [Tx;Ty];

end