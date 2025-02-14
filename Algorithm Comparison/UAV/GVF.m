%% This GVF in Yao 2020 Automatica is for path following the circle of r2, centered at the origin
function Chi = GVF(p)
    k1 = 0.16;
    global r2
    Phi = p(1)^2 + p(2)^2 -r2^2;
    Chi = [-2*p(2);2*p(1)] - k1 * Phi * 2*p;
    Chi = Chi/norm(Chi);
end