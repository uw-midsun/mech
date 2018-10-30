% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - computeSuspensionError.m
% A function that returns the length of a triangle's legs given the angle
% opposite to the desigred leg and the lengths of the other two legs. 
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2016
%
%
% Notes:
%
%   - Uses the cosine law: http://hyperphysics.phy-astr.gsu.edu/hbase/lcos.html
%
%   - angleC is the known angle opposite of leg c in degrees
%
%   - a and b are the other two legs of known length
%
% ---------------------------------------------------------------------- %

function c = cosLawGetOppositeLength(a,b,angleC)
    c = sqrt(a^2+b^2-(2*a*b*cosd(angleC)));
end