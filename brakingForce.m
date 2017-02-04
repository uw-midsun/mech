% ---------------------------------------------------------------------- %
%
% Longitudinal Weight Balance - longWeightBalance.m
% Script to compute the longitudinal weight balance of the car during
% braking or acceleration.
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

close all
clear all

% --------------------------- Constants ------------------------------- %

m = 611;            % car mass in kg
cogX = 0.8;         % Center of gravity position in longitudinal direction
cogY = 0.5;         % Center of gravity position in vertical direction
wheelbase = 1.78;   % Wheelbase in m
g = -9.81;          % Acceleration of gravity (m/s^2)
muBrakePad = 0.35;  % Coefficient of friction between the tire and road
muRoad = 0.6;       % Coefficient of friction between the tire and road

a = 5;              % Target acceleration
frontWeightBalance = .65; % Proportion of car's weight on front wheels druing hard
                          % braking
wheelDiam = 0.533;  % Wheel Diameter in m
rotorDiam = .18;    % Rotor diameter at center of brake pad
padContactArea = 0.00129 % Caliper Contact area in m^2 http://www.wilwood.com/BrakePads/BrakePadsProd.aspx?itemno=150-4091K
caliperCylinderArea = 0.001; % Area of caliper cylinder 
masterCylinderArea = 0.001; % Area of master cylinder 
brakePedalReduction = 3; % 3:1 mechanical advantage on brake pedal

% -------------------- Solving for Braking forces --------------------- %

% The dynamic of braking can be modeled by four linearly inpendnet
% equations: 
%       1) Sum of forces in the x = mass * car acceleration 
%       2) Sum of forces in the y = 0
%       3) Sum of the moments about the center of gravity = 0
%       4) The front braking force will be at max mu*normal force
% Expressing the following system of equations in maxtrix form yeilds Af=b
% where f is a vector containing: front brake force, rear brake force, 
% front normal force, rear normal force (in that order)

A = [1 1 0 0;
     0 0 1 1
     cogY cogY -cogX (wheelbase-cogX)
     1 0 -muRoad 0]
 
b = [m*a/2; -m*g/2; 0; 0];
 
F = linsolve(A,b)

weightBalance = (F(1)/(F(1)+F(2)))

for i = 1:2 
    torque(i) = F(i)*wheelDiam/2;
    brakeFriction(i) = torque(i)/(rotorDiam/2);
    brakePadForce(i) = brakeFriction(i)/muBrakePad;
    linePressure(i) = brakePadForce(i)/caliperCylinderArea
    brakePedalForce(i) = linePressure(i)*masterCylinderArea/brakePedalReduction
end