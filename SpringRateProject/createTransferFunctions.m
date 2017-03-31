% ---------------------------------------------------------------------- %
%
% Suspension Spring Rates Project - createTransferFuncitons.m
% Function setup transfer functions for a half car model to determine the
% reponse of MSXII to disturbances in the road conditions
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

load_system('halfCarModel');

% ----------------------------- Constants ------------------------------ %

c1_ = 1500;     % Front damping coefficient (Ns/m)
c2_ = 1600;     % Rear damping coefficient (Ns/m) 
k1_ = 51993;    % Front sring rate (N/m) 
k2_ = 30000;    % Rear sring rate (N/m)
m_ = 500;       % Mass of the vehicle (kg)
I_ = 550;       % Moment of interial about global x axis (kg*m^2)
a_ = 1.3;       % Distance from COG to front tire (m)
b_ = 1.3;       % Distance from COG to rear tire (m)
speed_ = 22.2;  % Cruising speed (m/s)

% ----------------- Create Symbolic Transfer Functions ----------------- %

syms s c1 c2 k1 k2 m I a b t0 

QA1 = m*s^2 + (c1+c2)*s + (k1+k2); % X terms of ODE (A) 
QA2 = (b*c2-a*c1)*s + (b*k2-a*k1); % Theta terms of ODE (A)
QB1 = (b*c2-a*c1)*s + (b*k2-a*k1); % X terms of ODE (B)
QB2 = I*s^2 + (b^2*c2+a^2*c1)*s + (b^2*k2+a^2*k1); % Theta terms of ODE (B)

PA1 = (c1*s+k1); % Time shifted componet of P for ODE (A) (front tire)
PA2 = (c2*s+k2); % Non time shifted componet of P for ODE (A) (rear tire)
PB1 = (-a*c1*s-a*k1); % Time shifted componet of P for ODE (B) (front tire)
PB2 = (b*c2*s+b*k2); % % Non time shifted componet of P for ODE (B) (rear tire)

% timeshift = exp(-s*t0); % Time shift to be applied to P_1

% Create linear systems in to solve for transfer functions

Q = [QA1, QA2;
     QB1, QB2];
 
P1 = [PA1;
      PB1];
P2 = [PA2;
      PB2];
 
H1 = linsolve(Q,P1);
H2 = linsolve(Q,P2);

% -------- Substitue constants to obtain numerial coefficients --------- %

numericalH1 = subs(H1, {c1 c2 k1 k2 m I a b}, {c1_ c2_ k1_ k2_ m_ I_ a_ b_});
numericalH2 = subs(H2, {c1 c2 k1 k2 m I a b}, {c1_ c2_ k1_ k2_ m_ I_ a_ b_});

[numH1, denomH1] = numden(numericalH1);
[numH2, denomH2] = numden(numericalH2);

transFuncA1 = tf(sym2poly(numH1(1)),sym2poly(denomH1(1)));
set_param('halfCarModel/HA1','Numerator',mat2str(sym2poly(numH1(1)))...
                            ,'Denominator',mat2str(sym2poly(denomH1(1))));

transFuncA2 = tf(sym2poly(numH2(1)),sym2poly(denomH2(1)));
set_param('halfCarModel/HA2','Numerator',mat2str(sym2poly(numH2(1)))...
                            ,'Denominator',mat2str(sym2poly(denomH2(1))));

transFuncB1 = tf(sym2poly(numH1(2)),sym2poly(denomH1(2)));
set_param('halfCarModel/HB1','Numerator',mat2str(sym2poly(numH1(2)))...
                            ,'Denominator',mat2str(sym2poly(denomH1(2))));
                        
transFuncB2 = tf(sym2poly(numH2(2)),sym2poly(denomH2(2)));
set_param('halfCarModel/HB2','Numerator',mat2str(sym2poly(numH2(2)))...
                            ,'Denominator',mat2str(sym2poly(denomH2(2))));
                        