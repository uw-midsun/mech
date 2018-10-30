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
%   - MSXII global coordinat system used
%   - Scalar verison of staticTrailingArmForces()
%   - Considers loading only in the static case
%   - Simplifies problem to 2D
%   - coiloverAngle is the angle that the coilover strut makes with the
%     central axis of the trailing arm
%   - trailingArmAngle is the angle the trailing arm makes with horizontal
% ---------------------------------------------------------------------- %

function [fMountY, fMountZ, fCoilover] = staticScalarTrailingArmForces(fRoadY, fRoadyZ, coiloverAngle,...
                                                       trailingArmAngle,... 
                                                       wheelMomentArm,...
                                                       coiloverLateralMomentArm,...
                                                       coiloverLongitudianalMomentArm)
                                                   
    fCoilover = (fRoadY.*cosd(trailingArmAngle) - fRoadyZ.*cosd(trailingArmAngle)).*wheelMomentArm /...
                (cosd(trailingArmAngle).*coiloverLongitudianalMomentArm +...
                 sind(coiloverAngle).*coiloverLateralMomentArm);
                
    fCoiloverZ = fCoilover*sind(coiloverAngle-trailingArmAngle);
    fCoiloverY = fCoilover*cosd(coiloverAngle-trailingArmAngle);
    
    fMountZ = fRoadyZ - fCoiloverZ;
    fMountY = fCoiloverY - fRoadY;
end