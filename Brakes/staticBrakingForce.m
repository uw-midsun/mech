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
% ---------------------------------------------------------------------- %

function force = staticBrakingForce(pressure, mu, area, rotorDiam, tireDiam)
    force = pressure.*area.*mu.*rotorDiam./tireDiam;
end

