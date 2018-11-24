
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

% --------------------------- Constants -------------------------------- %

% For this case, we assume Length = 2.25 m, Dc = -4.47, t = 3 

L = 2.25;               % Length of wheelbase
% Dc = -4.47;           % Deceleration of car
H = 1.5;                % Distance between ground and Center of Gravity 
k = 1;                  % index for arrays 
m = 720;                % Masss of Car in Kg with People
u = 0.4;                % Coefficient of Friction
g = 9.8;                % gravity

% --------------------------- Variables -------------------------------- %

A2 = zeros(1,40500);     % Deceleration 
Fbf = zeros(1,40500);    % Normal reaction force of front wheels
A1 = zeros(1,40500);     % position of COM 
               
%Cycle through FBF at various decelerations to determine COM positions
 
%Fbf = u*(((m*g*A1)+(m*A2*H))/L);

for a = -4: -0.125: -10
    for alpha = 0: 0.1: 2.25
        Fbf(k) = u*(((m*g*alpha)+(m*a*H))/L);
        A2(k)= a;
        A1(k)= alpha;
        k= k+1;
    end 
end 

        
plot3(A1,A2,Fbf)
title('CoM Solution Space \it(\mu = 0.4 (wet road), regular wheel)')
xlabel({'CoM Position','(m)'},'Color','r')
ylabel({'Deceleration','(ms^2)'},'Color','r')
zlabel({'Braking Force on Front Wheels','(kN)'},'Color','r')
grid on
grid minor





% Populating Nf values for all possilbe combinations of weight and alpha
% 
% for Mass = 700: 1: 900
% 
%    for A= 0.1: 0.00825 : 1.75
% Nf(k) = (((Mass* 9.81 * (Length - A ))+(Mass * Dc * H)) / Length)/1000;
% Mass2(k)= Mass;
% A2(k)= A;
% k = k +1;
%   end
% 
% end
% 
% % Plot for feasible space
% 
% 
% subplot(2,1,1)
% plot3(A2, (Mass2*.7), Nf)
% 
% title('CoM Solution Space \it(\mu = 0.7 (dry road), regular wheel)')
% xlabel({'CoM Position','(m)'},'Color','r')
% ylabel({'Total mass of car','(kg)'},'Color','r')
% zlabel({'Braking Force on Front Wheels','(kN)'},'Color','r')
% grid on
% grid minor
% yticks(0:100:1000)
% xticks(0:.25:2)
% zticks(-2:1:6)
% 
% subplot(2,1,2)
% plot3(A2, (Mass2*.4), Nf)
% 
% title('CoM Solution Space \it(\mu = 0.4 (wet road), regular wheel)')
% xlabel({'CoM Position','(m)'},'Color','r')
% ylabel({'Total mass of car','(kg)'},'Color','r')
% zlabel({'Braking Force on Front Wheels','(kN)'},'Color','r')
% grid on
% grid minor
% yticks(0:100:1000)
% xticks(0:.25:2)
% zticks(-2:1:6)
