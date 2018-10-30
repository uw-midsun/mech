% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - getCamber.m
% A function that returns the camber angle in degrees 
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2016
%
%
% Notes:
%
%   - Negative camber occurs when the top of the wheel is close to the
%     center of the car than the bottom
% 
% ---------------------------------------------------------------------- %

function camber = getCamber(nodes)
    camber = atan2d(nodes(8,1)-(nodes(6,1)),(nodes(6,2)-nodes(8,2)));
end 

