% ---------------------------------------------------------------------- %
%
% Suspension - uprightForces.m
% Function to compute the forces acting on the upright 
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2017
%
%
% Notes:
%   
%   - Vector quantities are of the form [x; y; z]
%   - The function solves 6 equations 
%       - sum of the forces in all 3 axes = mass*accel - applied forces
%       - sum of the moments in all 3 axes = moment of inertia*accel -
%       applied moments
% ---------------------------------------------------------------------- %

function [fUCA, fLCA] = uprightForces(mass, moi, cog, linearAccel,...
                                      angularAccel, fRoad, posUCA, posLCA,... 
                                      posContact)
    % Compute moment arms
    rUCA = posUCA - cog;
    rLCA = posLCA - cog;
    rRoad = posContact - cog;
    % Compute weight vector
    weight = mass.*[0;-9.81;0];
    % Setup linear equations
    A = [ 1        0        0        1        0        0
          0        1        0        0        1        0
          0        0        1        0        0        1
          0        -rUCA(3) rUCA(2)  0        -rLCA(3) rLCA(2)
          rUCA(3)  0        -rUCA(1) rLCA(3)  0        -rLCA(1)
          -rUCA(2) rUCA(1)  0        -rLCA(2) rLCA(1)  0 ];
    b = [(mass.*linearAccel - fRoad - weight); 
         (moi.*angularAccel - fRoad.*rRoad)];
    x = linsovle(A,b);
    % Parse results
    fUCA = x(1:3);
    fLCA = x(4:6);

end