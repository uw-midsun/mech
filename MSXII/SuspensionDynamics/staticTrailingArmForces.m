% ---------------------------------------------------------------------- %
%
% Suspension - staticTrailingArmForces.m
% Function to compute the forces acting on the trailing arm 
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2017
%
%
% Notes:
%   
%   - Vectors are of the form [x; y; z] in the MSXII coordinate system
%   - Considers loading only in the static case
%   - Simplifies problem to 2D
%   - coiloverAngle is the angle that the coilover strut makes with the
%     central axis of the trailing arm
%   - trailingArmAngle is the angle the trailing arm makes with horizontal
% ---------------------------------------------------------------------- %

function [fMount, fCoilover] = staticTrailingArmForces(fRoad, coiloverAngle,...
                                                       trailingArmAngle,... 
                                                       wheelMomentArm,...
                                                       coiloverLateralMomentArm,...
                                                       coiloverLongitudianalMomentArm)
                                                   
    fCoiloverMag = (fRoad(2)*cosd(trailingArmAngle) - fRoad(3)*cosd(trailingArmAngle))*wheelMomentArm /...
                   (cosd(trailingArmAngle)*coiloverLongitudianalMomentArm +...
                    sind(coiloverAngle)*coiloverLateralMomentArm);
                
    fCoilover(3) = fCoiloverMag*sind(coiloverAngle-trailingArmAngle);
    fCoilover(2) = fCoiloverMag*cosd(coiloverAngle-trailingArmAngle);
    
    fMount = [0;0;0];
    fMount(3) = fRoad(3) - fCoilover(3);
    fMount(2) = fCoilover(2) - fRoad(2);
end