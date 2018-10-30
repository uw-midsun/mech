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
%   - All force vectors are of the form [front, rear]
%
%   - Sliding condiditons: 
%       - 0: No sliding
%       - 1: Front sliding
%       - 2: Rear sliding
%       - 3: both sliding
%
% ---------------------------------------------------------------------- %

close all
clear all

% --------------------------- Constants ------------------------------- %

m = 550;            % car mass in kg
cogZ = 1.52;         % Center of gravity position in longitudinal direction (wrt global origin)
cogY = 0.52;         % Center of gravity position in vertical direction (wrt global origin)
wheelbase = 2.6;   % Wheelbase in m
g = -9.81;          % Acceleration of gravity (m/s^2)
muBrakePad = [0.35, 0.4];  % Coefficient of friction between the tire and road [dynamic, static]
muRoad = [0.6, 0.7];       % Coefficient of friction between the tire and road [dynamic, static]
dynamic = 1; % Constant to increase code readabliltiy 
static = 2; % Constant to increase code readabliltiy 
wheelDiam = 0.533;  % Wheel Diameter in m
rotorDiam = [.18, .3];    % Rotor diameter at center of brake pad [front, rear]
padContactArea = 0.00129; % Caliper Contact area in m^2 http://www.wilwood.com/BrakePads/BrakePadsProd.aspx?itemno=150-4091K
caliperCylinderArea = 0.001; % Area of caliper cylinder 
masterCylinderArea = 0.001; % Area of master cylinder 
brakePedalReduction = 3; % 3:1 mechanical advantage on brake pedal

% --------------------------- Variables ------------------------------- %

frontPressureRange = 5e5:5e5:20e6;
pressureRatioRange = 0:.05:1;

[frontPressure, pressureRatio] = meshgrid(frontPressureRange, pressureRatioRange);

% ----------------------- Solving for Reaction ------------------------- %

% Declare empty matricies to store results; 
a = zeros(length(pressureRatioRange),length(frontPressureRange));
weightBalance = a; 
sliding = a;
rearForce = a; 

for i = 1:length(pressureRatioRange)
    for j = 1:length(frontPressureRange)
        currFrontPressure = frontPressure(i,j); % in Pascals
        currPressureRatio = pressureRatio(i,j); % rearPressure / frontPressure
        pressure = [currFrontPressure, currFrontPressure*currPressureRatio];
        
        % Case 1: No lockup
        brakingForce = staticBrakingForce(pressure, muBrakePad(dynamic), padContactArea, rotorDiam, wheelDiam);
        [normalForce, accel] = roadReactionForce(muRoad(dynamic), 0, brakingForce, wheelbase, cogZ, cogY, m, g);
        sliding(i,j) = 0;
        
        % Check for lockup
        frontSliding = brakingForce(1) > normalForce(1)*muRoad(static); 
        rearSliding = brakingForce(2) > normalForce(2)*muRoad(static);

        % Recalculate reaction forces
        if (frontSliding && ~rearSliding) % if front sliding and rear rolling
            [normalForce, accel] = roadReactionForce(muRoad(dynamic), 1, brakingForce, wheelbase, cogZ, cogY, m, g);
            rearSliding = brakingForce(2) > normalForce(2)*muRoad(static);
            sliding(i,j) = 1;
        elseif (~frontSliding && rearSliding)  % if front rolling and rear sliding
            [normalForce, accel] = roadReactionForce(muRoad(dynamic), 2, brakingForce, wheelbase, cogZ, cogY, m, g);
            frontSliding = brakingForce(1) > normalForce(1)*muRoad(static); 
            sliding(i,j) = 2;
        end

        % Recalculate reaction forces if both sliding
        if (frontSliding && rearSliding)  % if front sliding and rear sliding
            [normalForce, accel] = roadReactionForce(muRoad(dynamic), 3, brakingForce, wheelbase, cogZ, cogY, m, g);
            sliding(i,j) = 3;
        end

        % Save results
        a(i,j) = accel; 
        weightBalance(i,j) = (normalForce(1)/(sum(normalForce)));
        rearForce(i,j) = normalForce(2);
    end
end

% Plot
figure
mesh(frontPressure, pressureRatio, a.*-1, weightBalance);

% Formatting
scale = colorbar;
scale.Label.String = 'Weight Balance (front/total)';
xlabel('Front Pressure [Pa]');
zlabel('Acceleration [m/s^2]');
title('Acceleration as a Function of Brake Force');
ylabel('Pressure Ratio (rear/front)');

figure
mesh(frontPressure, pressureRatio, a.*-1, sliding);

% Formatting
scale = colorbar;
scale.Label.String = 'Sliding Condition';
xlabel('Front Pressure [Pa]');
zlabel('Acceleration [m/s^2]');
title('Acceleration as a Function of Brake Force');
ylabel('Pressure Ratio (rear/front)');
