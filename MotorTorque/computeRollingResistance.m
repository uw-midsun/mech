% ---------------------------------------------------------------------- %
%
% Motor Torque - computeRollingResistance.m
% Function to compute rolling resistance force (Newtons). Equation from 
% The Winning Solar Car (page 18)
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

function rollingResistance = computeRollingResistance(Cr, M, v)
    g = 9.81; % acceleration of gravity 
    rollingResistance = Cr.*g.*M.*(1+0.00173.*v);
end