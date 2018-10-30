% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - rotateCoords.m
% A function that rotates an input coordinate about a specified center of
% rotation by a specified angle in degrees
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2016
%
%
% Notes:
%
%   - Positive angles are CCW
% 
% ---------------------------------------------------------------------- %

function [x,y] = rotateCoords(inputCoords, centerOfRotation, angle)
    inputCoords = inputCoords - centerOfRotation;
    [theta,rho] = cart2pol(inputCoords(1),inputCoords(2));
    theta = theta + deg2rad(angle); 
    [outputCoords(1),outputCoords(2)] = pol2cart(theta,rho);
    outputCoords = outputCoords + centerOfRotation;
    x = outputCoords(1);
    y = outputCoords(2);
end 