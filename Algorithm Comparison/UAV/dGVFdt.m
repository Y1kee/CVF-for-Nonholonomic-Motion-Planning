function dot_T = dGVFdt(p,dot_p)
esp = 1e-6;
x = p(1); y = p(2);
pTpx = (GVF([x+esp;y])-GVF([x-esp;y]))/2/esp;
pTpy = (GVF([x;y+esp])-GVF([x;y-esp]))/2/esp;
J_T = [pTpx,pTpy];

dot_T = J_T*dot_p;
end