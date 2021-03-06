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
close all;

% ----------------------------- Constants ------------------------------ %

c1_ = 9670;     % Front damping coefficient (Ns/m)
c2_ = 9670;     % Rear damping coefficient (Ns/m) 
k1_ = 650*175.1;    % Front sring rate (N/m) 
k2_ = 550*175.1;    % Rear sring rate (N/m)
m_ = 550;       % Mass of the vehicle (kg)
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
% set_param('halfCarModel/Road','Frequency',mat2str(wb),'Amplitude',mat2str(amplitude));
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

% -------------- Solve for response with zero damping ------------------ %

simplifiedH1 = simplify(collect(subs(H1, {k1 k2 c1 c2 m I a b}, {k1_ k2_ 0 0 m_ I_ a_ b_})));
simplifiedH2 = simplify(collect(subs(H2, {k1 k2 c1 c2 m I a b}, {k1_ k2_ 0 0 m_ I_ a_ b_})));
impulseResponse = vpa(simplify(ilaplace(simplifiedH1) + ilaplace(simplifiedH2*phaseShift)),3);

Fs = 200; % sampling frequency in Hz
T = 1/Fs; % sampling period
t_ = 0:T:30; % time values 0 to 30 seconds
L = length(t_);

% Numerical impulse response with zero damping
numericalImpulseResponse = double(subs(impulseResponse,t,t_));
figure;
hold on;
plot(t_,numericalImpulseResponse(1,:)./1000,'DisplayName','x_0');
ylabel('Linear Displacement (mm)');
yyaxis right
plot(t_,numericalImpulseResponse(2,:).*(180/pi),'DisplayName','\theta');
ylabel('Angular Displacement (deg)');
xlabel('Time (s)');
xlim([0 3]);
title('Impusle Response with Zero Damping');
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
title('FFT of Impusle Response with Zero Damping');
xlim([0 10]);
saveas(gcf,'fftImpResp','png');

% --------------- Find Criticl Damping Coefficients -------------------- %

simplifiedH1 = simplify(collect(subs(H1, {k1 k2 m I a b}, {k1_ k2_ m_ I_ a_ b_})));
simplifiedH2 = simplify(collect(subs(H2, {k1 k2 m I a b}, {k1_ k2_ m_ I_ a_ b_})));

Fs = 100; % sampling frequency in Hz
T = 1/Fs; % sampling period
t_ = 0:T:3; % time values 0 to 5 seconds
L = length(t_);

% Spring rates to test:
dampingOptions = (100000:500:100000); % N/m
[c1Options, c2Options] = meshgrid(dampingOptions, dampingOptions);

% Preallocate outputs:
pitchSettlingTime = zeros(size(c2Options));
xSettlingTime = pitchSettlingTime;

for i = 1:size(c1Options,1)
    for j = 1:size(c1Options,2)     
        
        x1 = subs(simplify(collect(simplifiedH1(1))),{c1 c2},{c1Options(i,j) c2Options(i,j)});
        x2 = subs(simplify(collect(simplifiedH2(1))),{c1 c2},{c1Options(i,j) c2Options(i,j)});
        xWithSpring = vpa(simplify(simplify(ilaplace(simplify(x1.*(0.025/s)))) + simplify(ilaplace(simplify(x2.*(0.025/s))))),3);
        
        pitch1 = subs(simplify(collect(simplifiedH1(2))),{c1 c2},{c1Options(i,j) c2Options(i,j)});
        pitch2 = subs(simplify(collect(simplifiedH2(2))),{c1 c2},{c1Options(i,j) c2Options(i,j)});
        pitchWithSpring = vpa(simplify(simplify(ilaplace(simplify(pitch1.*(0.025/s)))) + simplify(ilaplace(simplify(pitch2.*(0.025/s).*phaseShift)))),3);
                
        % Compute response in the time domain
        xResponse = double(subs(xWithSpring,t,t_));
        pitchResponse = double(subs(pitchWithSpring,t,t_));
        
        % Record settling time
        xStepInfo = stepinfo(xResponse,t_,0.025);
        pitchStepInfo = stepinfo(pitchResponse,t_,0);
        xSettlingTime(i,j) = xStepInfo.SettlingTime;
        pitchSettlingTime(i,j) = pitchStepInfo.SettlingTime;
    end
end

figure;
surf(c1Options, c2Options, real(xSettlingTime));
figure;
surf(c1Options, c2Options, real(pitchSettlingTime));


% % --- Plot Magnitude of First Harmonic as a Funciton of Spring Rates --- %
% 
% simplifiedH1 = subs(H1, {c1 c2 m I a b}, {0 0 m_ I_ a_ b_});
% simplifiedH2 = subs(H2, {c1 c2 m I a b}, {0 0 m_ I_ a_ b_});
%
% Fs = 200; % sampling frequency in Hz
% T = 1/Fs; % sampling period
% t_ = 0:T:5; % time values 0 to 5 seconds
% L = length(t_);
% 
% % Spring rates to test:
% springRateOptions = (25000:20000:115000); % N/m
% [k1Options, k2Options] = meshgrid(springRateOptions, springRateOptions);
% 
% % Preallocate outputs:
% pitchHarmonicFrequency = zeros(size(k2Options));
% xHarmonicFrequency = pitchHarmonicFrequency;
% pitchHarmonicMagnitude = pitchHarmonicFrequency;
% xHarmonicMagnitude = pitchHarmonicFrequency;
% 
% for i = 1:size(k1Options,1)
%     for j = 1:size(k1Options,2)     
%         
%         x1 = subs(simplify(collect(simplifiedH1(1)*input)),{k1 k2},{k1Options(i,j) k2Options(i,j)});
%         x2 = subs(simplify(collect(simplifiedH2(1)*input)),{k1 k2},{k1Options(i,j) k2Options(i,j)});
%         xWithSpring = vpa(simplify(simplify(ilaplace(simplify(x1))) + simplify(ilaplace(simplify(x2)))),3);
%         
%         pitch1 = subs(simplify(collect(simplifiedH1(2)*input)),{k1 k2},{k1Options(i,j) k2Options(i,j)});
%         pitch2 = subs(simplify(collect(simplifiedH2(2)*input)),{k1 k2},{k1Options(i,j) k2Options(i,j)});
%         pitchWithSpring = vpa(simplify(simplify(ilaplace(simplify(pitch1))) + simplify(ilaplace(simplify(pitch2)))),3);
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
%         hold on
%         plot(f,p1);
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


