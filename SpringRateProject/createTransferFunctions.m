% ---------------------------------------------------------------------- %
%
% Suspension Spring Rates Project - createTransferFuncitons.m
% Function setup transfer functions for a half car model to determine the
% reponse of MSXII to disturbances in the road conditions
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2017
%
%
% Notes:
%   
%   - 
% ---------------------------------------------------------------------- %

load_system('halfCarModel');

% ----------------------------- Constants ------------------------------ %

c1_ = 0;     % Front damping coefficient (Ns/m)
c2_ = 0;     % Rear damping coefficient (Ns/m) 
k1_ = 50000;    % Front sring rate (N/m) 
k2_ = 50000;    % Rear sring rate (N/m)
m_ = 500;       % Mass of the vehicle (kg)
I_ = 550;       % Moment of interial about global x axis (kg*m^2)
a_ = 1.3;       % Distance from COG to front tire (m)
b_ = 1.3;       % Distance from COG to rear tire (m)

% ---------------------- Forcing Function Parameters ------------------- %

amplitude = 0.015;              % Amplitude of bumps on road surface
bumpSparation = 2;              % Distance between peaks on the road surface (m)
speed = 22.2;                   % Cruising speed (m/s)
wb = 2*pi*speed/bumpSparation;  % Frequency of base excitation (rad/sec)
timeDelay = (a_+b_)/speed;        % delay between input striking front and
                                % rear wheels (s)
                                
% Update simulink parameters
set_param('halfCarModel/Road','Frequency',mat2str(wb),'Amplitude',mat2str(amplitude));
set_param('halfCarModel/DelayA','DelayTime',mat2str(timeDelay));
set_param('halfCarModel/DelayB','DelayTime',mat2str(timeDelay));
                              
% ----------------- Create Symbolic Transfer Functions ----------------- %

syms t s c1 c2 k1 k2 m I a b t0 A w0 

QA1 = m*s^2 + (c1+c2)*s + (k1+k2); % X terms of ODE (A) 
QA2 = (b*c2-a*c1)*s + (b*k2-a*k1); % Theta terms of ODE (A)
QB1 = (b*c2-a*c1)*s + (b*k2-a*k1); % X terms of ODE (B)
QB2 = I*s^2 + (b^2*c2+a^2*c1)*s + (b^2*k2+a^2*k1); % Theta terms of ODE (B)

PA1 = (c1*s+k1); % Time shifted componet of P for ODE (A) (front tire)
PA2 = (c2*s+k2); % Non time shifted componet of P for ODE (A) (rear tire)
PB1 = (-a*c1*s-a*k1); % Time shifted componet of P for ODE (B) (front tire)
PB2 = (b*c2*s+b*k2); % % Non time shifted componet of P for ODE (B) (rear tire)

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
phaseshift = exp(-s*timeDelay); % Time shift to be applied to P_2
input = (amplitude*wb)/(s^2+wb^2); % Laplace transform of input L{A*sin(wb*t))}

% -------------- Solve for response with zero damping ------------------ %

simplifiedH1 = subs(H1, {c1 c2 m I a b}, {0 0 m_ I_ a_ b_});
simplifiedH2 = subs(H2, {c1 c2 m I a b}, {0 0 m_ I_ a_ b_});


%%%% testing %%%%

newH1 = subs(simplifiedH1,{k1 k2},{k1_ k2_});
newH2 = subs(simplifiedH2,{k1 k2},{k1_ k2_});
xLaplace = simplify(newH1(1)*input + newH2(1)*phaseshift*input);
[xLaplaceNum, xLaplaceDenom] = numden(xLaplace);
harmonics = unique(abs(imag(double(solve(xLaplaceDenom, s)))));

Fs = 200; % sampling frequency in Hz
T = 1/Fs; % sampling period
t_ = 0:T:5; % time values 0 to 5 seconds
L = length(t_);

x = ilaplace(simplifiedH1(1)*input) + ilaplace(simplifiedH2(1)*phaseshift*input);
xWithSpring = subs(x,{k1 k2},{k1_ k2_});

% Compute response in the time domain
xResponse = double(subs(xWithSpring,t,t_));

% Compute the first harmonic of the displacement
fftOut = fft(xResponse);
p2 = abs(fftOut/L);
p1 = p2(1:L/2+1);
p1(2:end-1) = 2*p1(2:end-1);
f = Fs*(0:(L/2))/L;
plot(f,p1);
[magnitude, harmonicIndex] = max(p1);
xHarmonicFrequency = f(harmonicIndex);
xHarmonicMagnitude = magnitude;
%%%% testing %%%%        

x = ilaplace(simplifiedH1(1)*input) + ilaplace(simplifiedH2(1)*phaseshift*input);
pitch = ilaplace(simplifiedH1(2)*input) + ilaplace(simplifiedH2(2)*phaseshift*input);

Fs = 200; % sampling frequency in Hz
T = 1/Fs; % sampling period
t_ = 0:T:5; % time values 0 to 5 seconds
L = length(t_);

% Spring rates to test:
springRateOptions = (25000:10000:115000); % N/m
[k1Options, k2Options] = meshgrid(springRateOptions, springRateOptions);

% Preallocate outputs:
% pitchHarmonicFrequency = zeros(size(k2Options));
% xHarmonicFrequency = pitchHarmonicFrequency;
% pitchHarmonicMagnitude = pitchHarmonicFrequency;
% xHarmonicMagnitude = pitchHarmonicFrequency;

% for i = 1:size(k1Options,1)
%     for j = 1:size(k1Options,2)     
%         xWithSpring = subs(x,{k1 k2},{k1Options(i,j) k2Options(i,j)});
%         pitchWithSpring = subs(pitch,{k1 k2},{k1Options(i,j) k2Options(i,j)});
%         
%         % Compute response in the time domain
%         xResponse = double(subs(xWithSpring,t,t_));
%         pitchResponse = double(subs(pitchWithSpring,t,t_));
%         
%         % Compute the first harmonic of the displacement
%         fftOut = fft(xResponse);
%         p2 = abs(fftOut/L);
%         p1 = p2(1:L/2+1);
%         p1(2:end-1) = 2*p1(2:end-1);
%         f = Fs*(0:(L/2))/L;
%         [magnitude, harmonicIndex] = max(p1);
%         xHarmonicFrequency(i,j) = f(harmonicIndex);
%         xHarmonicMagnitude(i,j) = magnitude;
%         
%         % Compute the first harmonic of the pitch
%         fftOut = fft(pitchResponse);
%         p2 = abs(fftOut/L);
%         p1 = p2(1:L/2+1);
%         p1(2:end-1) = 2*p1(2:end-1);
%         f = Fs*(0:(L/2))/L;
%         [magnitude, harmonicIndex] = max(p1);
%         pitchHarmonicFrequency(i,j) = f(harmonicIndex);
%         pitchHarmonicMagnitude(i,j) = magnitude;
% 
%     end
% end
% 
% figure;
% surf(k1Options, k2Options, xHarmonicFrequency);
% figure;
% surf(k1Options, k2Options, pitchHarmonicFrequency);
% figure;
% surf(k1Options, k2Options, xHarmonicMagnitude);
% figure;
% surf(k1Options, k2Options, pitchHarmonicMagnitude);

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


