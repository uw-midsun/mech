% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - getLateralScrub.m
% A function that returns the lateral scrub distance in mm 
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

function lateralScrub = getLateralScrub(nodes,origNodes)
    lateralScrub = nodes(8,1) - origNodes(8,1);
end 