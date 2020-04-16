function predictParticles = exampleHelperRobotStateTransition(pf, prevParticles, dT, u)
    thetas = prevParticles(:,3);
    v = u(1);
    w = u(2);
    l = length(prevParticles);
    % Generate velocity samples
    sd1 = 0.3;
    sd2 = 1.5;
    sd3 = 0.02;
    vh = v + (sd1)^2*randn(l,1);
    wh = w + (sd2)^2*randn(l,1);
    gamma = (sd3)^2*randn(l,1);
    % Add a small number to prevent div/0 error
    wh(abs(wh)<1e-19) = 1e-19;
    % Convert velocity samples to pose samples
    predictParticles(:,1) = prevParticles(:,1) - (vh./wh).*sin(thetas) + (vh./wh).*sin(thetas + wh*dT);
    predictParticles(:,2) = prevParticles(:,2) + (vh./wh).*cos(thetas) - (vh./wh).*cos(thetas + wh*dT);
    predictParticles(:,3) = prevParticles(:,3) + wh*dT + gamma*dT;
end
