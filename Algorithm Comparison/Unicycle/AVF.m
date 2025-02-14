function T = AVF(x,y)
%% F向量场，目标位置(0 0 0)，目标朝向x轴正向，输入参数

Fx = x^2 - y^2;
Fy = 2*x*y;

T = [Fx;Fy]/norm([Fx;Fy]);

end