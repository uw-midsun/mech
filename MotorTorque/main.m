% ---------------------------------------------------------------------- %
%
% Motor Torque - main.m
% Script to extimate motor torque requirements at vaious speeds, inclines,
% and accelerations
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

M = 550;        % Car mass (kg)
r = 0.267;      % Wheel radius (m)
Cr = 0.0055;    % Rolling resistance coefficient - Winning Solar Car pg 18
A = 1.8;        % Frontal area (m^2)
Cd = 0.16;      % Drag coefficient - from Ron's preliminary CFD findings
rho = 1.225;    % Density of air (kg/m^3)
g = 9.81;       % Acceleration of gravity (m/s^2)

% ------------------------------ Variables ---------------------------- %

a = [0:.5:2.5];                      % Acceleration in m/s^2
v = [0:.1:33.33];                    % Velocity in m/s
incline = [0, 5,10];            % Incline in %
theta = atan(incline/100);           % Incline in rad

% ---------------------------- Plot Results ---------------------------- %

figure;

for i = 1:length(theta)
    subplot(1,length(theta),i);
    hold on;
    for j = 1:length(a)
        torque = 0.5*r.*(M.*a(j) + M.*g.*sin(theta(i))...
                 + computeRollingResistance(Cr, M, v)...
                 + computeDrag(rho, v, Cd, A)); 
        plot(v,torque,'DisplayName', sprintf('%0.5g m/s^2',a(j)));
    end
    xlabel('Speed (m/s)');
    ylabel('Torque (Nm)')
    title(sprintf('%d%% incline',incline(i)))
    ylim([0,400]);
    grid on;
end
legend('show');


% --------------------- Plot Cornering Results ------------------------ %

% Save
% saveas(1,'outputs/Suspension Simulation Results - Cornering','png');
