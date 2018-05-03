% ---------------------------------------------------------------------- %
%
% High Level Car Model - computeDrag.m
% Function to compute drag force 
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2017
%
%
% Notes:
%   
%   - Units: 
%       Distance: meters
%       Mass: kilograms
%       Time: seconds
%       Angles: degrees
%
% ---------------------------------------------------------------------- %

function drag = computeDrag(rho, v, Cd, A)
    drag = 0.5.*rho.*(v.^2).*Cd.*A;
end