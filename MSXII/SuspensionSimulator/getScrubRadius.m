% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - getScrubRadius.m
% A function that returns the scrub radius in mm 
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2016
%
%
% Notes:
%
%   - 
% 
% ---------------------------------------------------------------------- %

function scrubRadius = getScrubRadius(nodes)
    dXdY = (nodes(3,1) - nodes(5,1)) / (nodes(3,2) - nodes(5,2));
    roadIntersectionPoint = nodes(5,1) - dXdY * nodes(5,2); 
    scrubRadius = nodes(8,1) - roadIntersectionPoint;
end 