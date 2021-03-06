% ---------------------------------------------------------------------- %
%
% Suspension Spring Rates Project - alternativeFormulation.m
% Function to setup transfer functions for a half car model to determine the
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
close all;
clear all

% ----------------------------- Constants ------------------------------ %

c1_ = 4830;     % Front damping coefficient (Ns/m)
c2_ = 4830;     % Rear damping coefficient (Ns/m) 
k1_ = 600*175.1*cosd(35);    % Front sring rate (N/m) 
k2_ = 500*175.1*cosd(30);    % Rear sring rate (N/m)
m_ = 550;       % Mass of the vehicle (kg)
I_ = 550;       % Moment of interial about global x axis (kg*m^2)
a_ = 1.32;       % Distance from COG to front tire (m)
b_ = 1.28;       % Distance from COG to rear tire (m)

% ---------------------- Forcing Function Parameters ------------------- %

amplitude = 0.015;              % Amplitude of bumps on road surface
bumpSparation = 2;              % Distance between peaks on the road surface (m)
speed = 22.2;                   % Cruising speed (m/s)
wb = 2*pi*speed/bumpSparation;  % Frequency of base excitation (rad/sec)
timeDelay = (a_+b_)/speed;      % delay between input striking front and
                                % rear wheels (s)
                                
% Update simulink parameters
set_param('halfCarModel/DelayA','DelayTime',mat2str(timeDelay));
set_param('halfCarModel/DelayB','DelayTime',mat2str(timeDelay));
                              
% ----------------- Create Symbolic Transfer Functions ----------------- %

syms t s c1 c2 k1 k2 m I a b t0 A w0 c k 

QA1 = m*s^2 + (c1+c2)*s + (k1+k2); % X terms of ODE (A) 
QA2 = (b*c2-a*c1)*s + (b*k2-a*k1); % Theta terms of ODE (A)
QB1 = (b*c2-a*c1)*s + (b*k2-a*k1); % X terms of ODE (B)
QB2 = I*s^2 + (b^2*c2+a^2*c1)*s + (b^2*k2+a^2*k1); % Theta terms of ODE (B)

PA1 = (c1*s+k1); % Non time shifted componet of P for ODE (A) (front tire)
PA2 = (c2*s+k2); % Time shifted componet of P for ODE (A) (rear tire)
PB1 = (-a*c1*s-a*k1); % Non time shifted componet of P for ODE (B) (front tire)
PB2 = (b*c2*s+b*k2); % % Time shifted componet of P for ODE (B) (rear tire)

% Create linear systems in to solve for transfer functions

Q = [QA1, QA2;
     QB1, QB2];
 
P1 = [PA1;
      PB1];
P2 = [PA2;
      PB2];
 
H1 = linsolve(Q,P1);
H2 = linsolve(Q,P2);

% Input in frequency domain
phaseShift = exp(-s*timeDelay); % Time shift to be applied to P_2
input = (amplitude*wb)/(s^2+wb^2); % Laplace transform of input L{A*sin(wb*t))}

% -------------------------- Solve for response  ----------------------- %

simplifiedH1 = simplify(collect(subs(H1, {k1 k2 c1 c2 m I a b}, {k1_ k2_ c1_ c2_ m_ I_ a_ b_})));
simplifiedH2 = simplify(collect(subs(H2, {k1 k2 c1 c2 m I a b}, {k1_ k2_ c1_ c2_ m_ I_ a_ b_})));
impulseResponse = vpa(simplify(ilaplace(simplifiedH1) + ilaplace(simplifiedH2*phaseShift)),3);

Fs = 200; % sampling frequency in Hz
T = 1/Fs; % sampling period
t_ = 0:T:30; % time values 0 to 30 seconds
L = length(t_);

% Numerical impulse response with zero damping
numericalImpulseResponse = double(subs(impulseResponse,t,t_));
figure;
hold on;
plot(t_,numericalImpulseResponse(1,:).*1000,'DisplayName','x_0');
ylabel('Linear Displacement (mm)');
yyaxis right
plot(t_,numericalImpulseResponse(2,:).*(180/pi),'DisplayName','\theta');
ylabel('Angular Displacement (deg)');
xlabel('Time (s)');
xlim([0 1]);
title('Impusle Response');
legend show;
saveas(gcf,'impResp','png');

% Compute the first harmonic of the displacement
fftOut = fft(numericalImpulseResponse(1,:));
p2 = abs(fftOut/L);
p1 = p2(1:L/2+1);
p1(2:end-1) = 2*p1(2:end-1);
f = Fs*(0:(L/2))/L;
figure;
hold on
plot(f,p1,'DisplayName','x_0');

% FourierTransform
fftOut = fft(numericalImpulseResponse(2,:));
p2 = abs(fftOut/L);
p1 = p2(1:L/2+1);
p1(2:end-1) = 2*p1(2:end-1);
f = Fs*(0:(L/2))/L;
plot(f,p1,'DisplayName','\theta');

legend show 
xlabel('Frequency (Hz)');
title('FFT of Impusle Response');
xlim([0 10]);
saveas(gcf,'fftImpResp','png');

% --------------------------------- Forces ----------------------------- %


% ------------------------------- Simulink ----------------------------- %

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

% --------------- Solving for natural resonance frequency -------------- %


