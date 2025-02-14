function T = CLVF(p,rho)
global r2
r2 = 8*rho;
r_d = r2;
r = norm(p);
x = p(1); y = p(2);

if r < r_d
    c = r/r_d;
else
    c = r_d/r;
end

T_x = -((r^2-r_d^2)*x/r + c*r_d*y);
T_y = -((r^2-r_d^2)*y/r - c*r_d*x);
T = [T_x;T_y]/norm([T_x;T_y]);
end