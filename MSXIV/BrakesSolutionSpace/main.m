% ---------------------------------------------------------------------- %
%
% Brakes & CoM Solution Space - main.m
% Script to develop solution space for front/back CoM position of MSXIV
% 
% Author: Neeraj Nair
% 
% © Midnight Sun 2018
%
%
% Notes:
%
%   - Units: 
%       Distance: millimeters
%       Mass: kilograms
%       Time: seconds
%       Angles: degrees
%
% ---------------------------------------------------------------------- %

Length = 0; % Length of wheelbase 
a = 0; % Distance of Center of Gravity from front wheel
Mass = 0; % Mass of car
Nf = 0; % Normal reaction force of front wheels
Nr = 0; % Normal reaction force of rear wheels 
Dc = 0; % Deceleration of car 
t= 0; % Braking time of car 
h = 0; % Distance between ground and Center of Gravity 
 
 
% For this case, we assume Length = 2.25 m, Dc = -4.47, t = 3 

Length = 2.25;
Dc = -4.47;
time  = 3;
h = 1.5;


syms Mass Nf Nr
Nr = ((Mass * 9.81) - Nf) /1000

 
Nf = (((Mass * 9.81 * (Length - a)) + (Mass * Dc * h))/Length)/1000
 
n = 10;
m = .25;
% o =10;


% % [xx,yy,zz] = meshgrid(0:n:800,0:m:2.25);
% 
% xx = Mass;
% yy = a; 
