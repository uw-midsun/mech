% ---------------------------------------------------------------------- %
%
% Suspension - lowerControlArmForces.m
% Function to compute the forces acting on the lower control arm 
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2017
%
%
% Notes:
%   
%   - Vector quantities are of the form [x; y; z]
% ---------------------------------------------------------------------- %

function [fPushrod] = pushRodForce(fUpright, pHardpoint, pUpright, p)