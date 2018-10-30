% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - computeDistance.m
% A function that returns the distance between two cartesian points
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2016
%
%
% Notes:
%
%   - 
% ---------------------------------------------------------------------- %

function d = computeDistance(x1,y1,x2,y2)
    d = sqrt((x1-x2)^2 + (y1-y2)^2);
end 