% ---------------------------------------------------------------------- %
%
% Suspension Spring Rates Project - simplifiedFormulation.m
% Function to setup simplified transfer functions for a half car model to 
% determine the reponse of MSXII to disturbances in the road conditions
% 
% Author: Devon Copeland
% 
% � Midnight Sun 2017
%
%
% Notes:
%   
%   - 
% ---------------------------------------------------------------------- %

close all;
clear all;
clc;

% ----------------------------- Constants ------------------------------ %

c_ = 4830;     % Front damping coefficient (Ns/m)
k_ = 88600;    % Front sring rate (N/m) 
M_ = 550;      % Mass of the vehicle (kg)
I_ = 550;      % Moment of interial about global x axis (kg*m^2)
a_ = 1.3;      % Distance from COG to front tire and rear tire (m)
zx_ = 0.49;     % Damping ratio of translational  
wnx_ = 17.95;   % Natural frequency of pitch
zth_ = 0.64;    % Damping ratio of pitch
wnth_ = 23.33;  % Natural frequency of pitch


% ---------------------- Forcing Function Parameters ------------------- %

A_ = 0.015;                     % Amplitude of bumps on road surface
bumpSparation = 2;              % Distance between peaks on the road surface (m)
speed = 22.2;                   % Cruising speed (m/s)
wb_ = 2*pi*speed/bumpSparation; % Frequency of base excitation (rad/sec)
t0_ = (2*a_)/speed;            % delay between input striking front and
                                % rear wheels (s)

% ----------------- Create Symbolic Transfer Functions ----------------- %                                
                                
syms t s M I a t0 A wb c k zx wnx zth wnth

QA = s^2 + 2*c/M*s + wnx^2; % ODE A 
QB = s^2 + 2*a^2*c/I*s + wnx^2; % ODE B

phaseShift = exp(-s*t0);

PA = [(c*A*wb/M)*(s/(s^2+wb^2));
	  (c*A*wb/M)*(s/(s^2+wb^2))*phaseShift;
	  (k*A/M)*(wb/(s^2+wb^2));
	  (k*A/M)*(wb/(s^2+wb^2))*phaseShift];

PB = [(-c*a*A*wb/I)*(s/(s^2+wb^2));
      (c*a*A*wb/I)*(s/(s^2+wb^2))*phaseShift;
      (-k*a*A/I)*(wb/(s^2+wb^2));
      (k*a*A/I)*(wb/(s^2+wb^2))*phaseShift];
  
X = collect((PA./QA),s);
Theta = collect((PB./QB),s);

subX = simplify(subs(X, {M I a t0 A wb c k zx wnx zth wnth},{M_ I_ a_ t0_ A_ wb_ c_ k_ zx_ wnx_ zth_ wnth_}));
subTheta = simplify(subs(Theta, {M I a t0 A wb c k zx wnx zth wnth},{M_ I_ a_ t0_ A_ wb_ c_ k_ zx_ wnx_ zth_ wnth_}));

x = simplify(collect(sum(ilaplace(subX))));
theta = simplify(collect(sum(ilaplace(subTheta))));

% Print total response 
latex(vpa(x,3))
latex(vpa(theta,3))

totalResponse = [x;theta];

% Plot total response
t_ = 0:0.005:1;
numericalResponse = double(subs(totalResponse,t,t_));
figure;
hold on;
plot(t_,numericalResponse(1,:).*1000,'DisplayName','x_0');
ylabel('Linear Displacement (mm)');
yyaxis right
plot(t_,numericalResponse(2,:).*(180/pi),'DisplayName','\theta');
ylabel('Angular Displacement (deg)');
xlabel('Time (s)');
xlim([0 1]);
title('Total Response');
legend show;
saveas(gcf,'totalResp','png');
