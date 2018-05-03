% ---------------------------------------------------------------------- %
%
% High Level Car Model - rungeKutta4.m
% Performs one iteration of 4th order runge kutta
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2018
%
% Notes:
%   
%   - The parameter fcn is a differential equation of the 
%     form y' = fcn(t,y)
%
% ---------------------------------------------------------------------- %

function yNext = rungeKutta4(fcn, t, deltat, y)
    k1 = deltat * fcn(t, y);
    k2 = deltat * fcn(t+deltat/2, y+k1/2);
    k3 = deltat * fcn(t+deltat/2, y+k2/2);
    k4 = deltat * fcn(t+deltat, y+k3);
    yNext = y + (k1 + 2*k2 + 2*k3 + k4)/6; 
end