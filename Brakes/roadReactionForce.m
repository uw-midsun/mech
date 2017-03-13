% ---------------------------------------------------------------------- %
%
% Function that returns the brake force based for the line pressure, pad
% area, rotor diameter, and coefficient of friction
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
%       Pressure: pascals
%
% The dynamics of braking can be modeled by three linearly inpendnet
% equations: 
%       1) Sum of forces in the x = mass * car acceleration 
%       2) Sum of forces in the y = 0
%       3) Sum of the moments about the center of gravity = 0
% Expressing the following system of equations in maxtrix form yeilds Af=b
% where f is a vector containing: acceleration, a front brake force and 
% rear brake force (in that order)
%
% ---------------------------------------------------------------------- %

function [force, accel] = roadReactionForce(dynamicMuRoad, sliding, brakingForce, wheelbase, cogZ, cogY, m, g)
    if(sliding == 0)
        % Setup A matrix
        A = [m/2 0 0;
             0 1 1;
             0 -cogZ (wheelbase-cogZ)]; 
        % Setup b vector 
        b = [-(brakingForce(1)+brakingForce(2)); 
             -m*g/2; 
             -(brakingForce(1)*cogY+brakingForce(2)*cogY)];
         
    elseif(sliding == 1)
        % Setup A matrix
        A = [m/2 dynamicMuRoad 0;
             0 1 1;
             0 (-cogZ+cogY*dynamicMuRoad) (wheelbase-cogZ)]; 
        % Setup b vector 
        b = [-brakingForce(2); 
             -m*g/2; 
             -brakingForce(2)*cogY];
         
    elseif(sliding == 2)
        % Setup A matrix
        A = [m/2 0 dynamicMuRoad;
             0 1 1;
             0 -cogZ (wheelbase-cogZ+cogY*dynamicMuRoad)]; 
        % Setup b vector 
        b = [-brakingForce(1); 
             -m*g/2; 
             -brakingForce(1)*cogY];
         
    elseif(sliding == 3)
        % Setup A matrix
        A = [m/2 dynamicMuRoad dynamicMuRoad;
             0 1 1;
             0 (-cogZ+cogY*dynamicMuRoad) (wheelbase-cogZ+cogY*dynamicMuRoad)]; 
        % Setup b vector 
        b = [0; 
             -m*g/2; 
             0];
    end
    
    % Solve
    result = linsolve(A,b);
    % Parse
    accel = result(1);
    force(1) = result(2);
    force(2) = result(3); 
end

