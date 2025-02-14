function dot_g = DVF(t,g)
    theta = g(1);
    p = g(2:3);
    
    % Internal parameter theta
    dot_theta = -theta;

    % 2D DVF
    if theta ~= 0
        Chi = theta*(1+cos(theta))/(2*sin(theta));
    else
        Chi = 1;
    end
    phi = [Chi, theta/2; -theta/2, Chi] * p;
    Gamma = -ang2rtm(theta)*phi;
    
    % Time derivative of g
    dot_g = [dot_theta;Gamma];
end