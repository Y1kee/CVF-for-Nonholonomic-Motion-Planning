function kw = kw(q,theta,rho,v)
kw_max = 1;
vx = norm(v);
K = 1/rho;
r_delta = norm(q);
phi = atan2(q(2),q(1));
T = CVF(q,rho);

R = ang2rtm(theta);
theta_r = vec2ang(T);
R_r = ang2rtm(theta_r);
R_e = R_r'*R;
theta_e_wegde = logm(R_e);
theta_e = theta_e_wegde(2,1);

alpha = atan2(g(r_delta),1/r_delta);
beta = theta - phi +alpha;
k0 = (K-k(r_delta,rho)*abs(sin(beta)))/abs(theta_e)*vx;
kw = min(kw_max,k0);
end

function k = k(r_delta,rho)
if r_delta >= rho
    k = 1/r_delta + g(r_delta);
else
    k = r_delta/rho^2;
end
end

function g = g(r_delta)
global r1 r2 r3
if r_delta <=r1
    g = 0;
elseif r_delta <=r2
    s2 = (r_delta-r1)/(r2-r1);
    lambda = mix(s2);
    g = (6*s2-6*s2^2)/(r2-r1)/(2*lambda^2-2*lambda+1);
elseif r_delta<=r3
    s3 = (r_delta-r2)/(r3-r2);
    lambda = mix(s3);
    g = (6*s3-6*s3^2)/(r3-r2)/(2*lambda^2-2*lambda+1);
else
    g = 0;
end
end
