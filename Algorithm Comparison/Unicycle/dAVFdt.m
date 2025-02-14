%% Calculate the time derivative of the Dipole-like vector field
function dot_T = dAVFdt(x,y,dot_x,dot_y)
esp = 1e-6;
pTpx = (AVF(x+esp,y)-AVF(x-esp,y))/2/esp;
pTpy = (AVF(x,y+esp)-AVF(x,y-esp))/2/esp;
J_T = [pTpx,pTpy];

dot_p = [dot_x,dot_y]';
dot_T = J_T*dot_p;
end