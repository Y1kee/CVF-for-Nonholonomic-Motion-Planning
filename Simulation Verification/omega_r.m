function w_r = omega_r(p,p_delta,v)
q = p-p_delta;
r_delta = norm(q);
vx_paral = (p-p_delta)'*v/norm(p-p_delta);
vx_ortho = ([0,-1;1,0]*(p-p_delta))'*v/norm(p-p_delta);

w_r = 1*vx_ortho/r_delta + g(r_delta)*vx_paral;
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