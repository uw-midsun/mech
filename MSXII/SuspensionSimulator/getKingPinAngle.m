% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - getKingPinAngle.m
% A function that returns the king pin angle in degrees 
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

function kingPinAngle = getKingPinAngle(nodes)
    kingPinAngle = atan2d(nodes(5,1)-(nodes(3,1)),(nodes(3,2)-nodes(5,2)));
end 