% ---------------------------------------------------------------------- %
%
% SuspensionDynamics - trailingArmForces.m
% Script to extimate the forces on the trailing arm based on loads from the
% wheel 
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2017
%
%
% Notes:
%   
%   - Units: 
%       Distance: meters
%       Mass: kilograms
%       Time: seconds
%       Angles: degrees
%
% ---------------------------------------------------------------------- %

clear all;
close all;

% ------------------------------ Constants ---------------------------- %

coiloverAngle = 45; % angle that the coilover strut makes with the central axis of the trailing arm in degrees
trailingArmAngle = 15; % angle the trailing arm makes with horizontal in degrees
wheelMomentArm = 0.300; % (meters) distance between wheel center and pivot point
coiloverLateralMomentArm = 0.05;% (meters) distance between trailing arm central axis and coilover mounting point
coiloverLongitudianalMomentArm = 0.100; % (meters) distance along trailing arm centerline between pivot point and coilover mounting point

% ------------------------------ Variables ---------------------------- %

fRoadNormalRange = [0:50:(500*0.5*9.81)]; % Newtons
fRoadFricitonRange = [0:5:(60/(.5*.533))]; % Newtons

[fRoadNormal, fRoadFriciton] = meshgrid(fRoadNormalRange, fRoadFricitonRange);

% ------------------------------ Results ------------------------------ %

[fMountY, fMountZ, fCoilover] = staticScalarTrailingArmForces(fRoadNormal,... 
                                fRoadFriciton, coiloverAngle, trailingArmAngle,... 
                                wheelMomentArm, coiloverLateralMomentArm,...
                                coiloverLongitudianalMomentArm);
                           
figure;
hold on;
surf(fRoadNormal, fRoadFriciton, fCoilover);
surf(fRoadNormal, fRoadFriciton, fMountY);
surf(fRoadNormal, fRoadFriciton, fMountZ);

