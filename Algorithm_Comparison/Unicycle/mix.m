function lambda = mix(x)
%% Blendding function smooth on decrease from 1 to 0 on [0,1]
lambda = 2*x^3-3*x^2+1;
% lambda = 1-cos(x*pi/2)^2;
end