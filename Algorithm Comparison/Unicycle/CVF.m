function T = CVF(p,rho)
global r1 r2 r3
r = norm(p);

if r <= r1
    T = source(p);
elseif r <= r2
    lambda = mix((r-r1)/(r2-r1));
    T = normalize(lambda*source(p) + (1-lambda)*vortex(p),'norm');
elseif r <= r3
    lambda = mix((r-r2)/(r3-r2));
    T = normalize(lambda*vortex(p) + (1-lambda)*sink(p),'norm');
else
    T = sink(p);
end

end