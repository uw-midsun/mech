% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - main.m
% Script to simulate the suspension geometry of MSXII
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2016
%
%
% Notes:
%
%   - Coordinte system follows standard for MSXII: 
%     https://uwmidsun.atlassian.net/wiki/display/MECH/MSXII
%   
%   - Units: 
%       Distance: millimeters
%       Mass: kilograms
%       Time: seconds
%       Angles: degrees
%
% ---------------------------------------------------------------------- %

clear all;
close all;

% ------------------------------ Config -------------------------------- %

% Plots
VISUALIZER_ON = true;
PLOT_ROLL_CENTER_GEOMETRY = false;

% Longitudial shift
START_DISP = 0;
END_DISP = 100;
DISP_STEP = 5;
NEUTRAL_DISP = 0;

% Cornering (roll)
ROLL_STEP = .01;
END_ROLL = 4; 

% Override 
if PLOT_ROLL_CENTER_GEOMETRY
    VISUALIZER_ON = true;
end

% --------------------------- Input Parameters ------------------------- %

% Vector contianing the parameters which define the suspension geometry
parametersRight = [ 1199.0;   % UCA mount (x)
                    600.0;    % UCA mount (y)
                    999.0;    % LCA mount (x)
                    199.0;    % LCA mount (y)
                    250.0;    % UCA length
                    500.0;    % LCA length
                    450.0;    % upright length
                    157.0;    % spindle height
                    130.0;    % wheel offset
                    700.0;    % wheel diameter
                    83   ];   % spindle angle

% Generate parameters for left wheel
parametersLeft = parametersRight; 
parametersLeft(1) = parametersLeft(1) * -1;
parametersLeft(3) = parametersLeft(3) * -1;

% ------------------------- Initialize Geometry ------------------------ %

% An approximation of double wishbode suspension to give fsolve a good
% place to start:
% Each row represents a node: (x,y)
nodesGuessRight = [ 1000, 600;    % UCA mount
                    1000, 200;    % LCA mount
                    1300, 600;    % upright top
                    1300, 400;    % upright spindle mount
                    1300, 200;    % upright btm
                    1500, 800;    % wheel top
                    1500, 400;    % wheel center
                    1500, 0    ]; % wheel btm 

nodesGuessLeft = nodesGuessRight;
nodesGuessLeft(:,1) = nodesGuessLeft(:,1) .* -1;
           
% Matrix to map nodes to elements in the suspension, the ith row stores the
% nodes connecting the ith element
elements = [ 1,3;
             2,5;
             3,4;
             4,5;
             4,7;
             6,7;
             7,8  ]; 

% ---------- Solve suspension motion (longitudinal COG shift) ---------- %

% Index to log results
i = 1; 

for disp = START_DISP:DISP_STEP:END_DISP
    
    % Compute right nodal positions
    [solvedNodes,fval] = fsolve(@(uNodes) computeSuspensionError(uNodes, ...
                                parametersRight), extractUnknownNodes(nodesGuessRight)); 
    nodesRight = assembleSolution(solvedNodes, parametersRight);

    % Compute left nodal positions
    [solvedNodes,fval] = fsolve(@(uNodes) computeSuspensionError(uNodes, ...
                                parametersLeft), extractUnknownNodes(nodesGuessLeft)); 
    nodesLeft = assembleSolution(solvedNodes, parametersLeft);
        
    % Log neutral position
    if disp == NEUTRAL_DISP
        origNodesRight = nodesRight;
        origNodesLeft = nodesLeft;
    end
    
    % Visualization
    if VISUALIZER_ON
        upperYLim = max(max(nodesLeft(:,2),nodesRight(:,2)));
        limits = [min(nodesLeft(:,1)) * 1.2, max(nodesRight(:,1)) * 1.2; 
                  -0.1 * upperYLim, upperYLim * 1.2];
        figure(2);
        clf(2);
        axis equal;
        printSuspension(origNodesRight,nodesRight,elements,limits);
        printSuspension(origNodesLeft,nodesLeft,elements,limits);
        [rollCentX, rollCentY] = getRollCenter(nodesRight, nodesLeft, ...
            PLOT_ROLL_CENTER_GEOMETRY, limits); % Redundant call for visualizer only
        plot(rollCentX, rollCentY,'k^');
        legend('show')
        printLabels(limits);
        saveas(2,sprintf('outputs/down%d',disp),'png');
        if disp > START_DISP && disp < END_DISP
            saveas(2,sprintf('outputs/up%d',END_DISP-disp),'png');
        end
    end
    
    % Log results
    longitudinalResults(i).disp = disp; 
    longitudinalResults(i).camber = getCamber(nodesRight);
    longitudinalResults(i).lateralScrub = getLateralScrub(nodesRight,origNodesRight);
    longitudinalResults(i).kingPinAngle = getKingPinAngle(nodesRight);
    longitudinalResults(i).scrubRadius = getScrubRadius(nodesRight);
    [longitudinalResults(i).rollCenterX longitudinalResults(i).rollCenterY] ...
        = getRollCenter(nodesRight, nodesLeft, false, limits);
    
    % Update Parameters
    parametersRight(2) = origNodesRight(1,2) + disp;
    parametersRight(4) = origNodesRight(2,2) + disp;
    parametersLeft(2) = origNodesLeft(1,2) + disp;
    parametersLeft(4) = origNodesLeft(2,2) + disp;
    
    % Incrament counter
    i = i + 1;
end

% ---------- Solve suspension motion (roll/Cornering) ---------- %

% Index to log results
i = 1; 

% Array of angles to iterate on
rollIncraments = [0:ROLL_STEP:END_ROLL, ... % CCW
                  (END_ROLL-ROLL_STEP):(-1*ROLL_STEP):(-1*END_ROLL), ... % CW
                  (-1*END_ROLL+ROLL_STEP):ROLL_STEP:(-1*ROLL_STEP)]; % CCW

prevRoll = rollIncraments(1);

for roll = rollIncraments
    
    % Compute right nodal positions
    [solvedNodes,fval] = fsolve(@(uNodes) computeSuspensionError(uNodes, ...
                                parametersRight), extractUnknownNodes(nodesGuessRight)); 
    nodesRight = assembleSolution(solvedNodes, parametersRight);

    % Compute left nodal positions
    [solvedNodes,fval] = fsolve(@(uNodes) computeSuspensionError(uNodes, ...
                                parametersLeft), extractUnknownNodes(nodesGuessLeft)); 
    nodesLeft = assembleSolution(solvedNodes, parametersLeft);
        
    % Log neutral position
    if i == 1
        origNodesRight = nodesRight;
        origNodesLeft = nodesLeft;
    end
        
    % Visualization (every 10 incraments)
    if VISUALIZER_ON && ~(mod(i,10))
        upperYLim = max(max(nodesLeft(:,2),nodesRight(:,2)));
        limits = [min(nodesLeft(:,1)) * 1.2, max(nodesRight(:,1)) * 1.2; 
                  -0.1 * upperYLim, upperYLim * 1.2];
        figure(2);
        clf(2);
        axis equal;
        printSuspension(origNodesRight,nodesRight,elements,limits);
        printSuspension(origNodesLeft,nodesLeft,elements,limits);
        [rollCentX, rollCentY] = getRollCenter(nodesRight, nodesLeft, ...
            PLOT_ROLL_CENTER_GEOMETRY, limits); % Redundant call for visualizer only
        plot(rollCentX, rollCentY,'k^');
        legend('show')
        printLabels(limits);
        saveas(2,sprintf('outputs/roll%d',i),'png');
    end
    
    % Log results
    rollResults(i).disp = roll; 
    rollResults(i).camber = getCamber(nodesRight);
    rollResults(i).lateralScrub = getLateralScrub(nodesRight,origNodesRight);
    rollResults(i).kingPinAngle = getKingPinAngle(nodesRight);
    rollResults(i).scrubRadius = getScrubRadius(nodesRight);
    [rollResults(i).rollCenterX, rollResults(i).rollCenterY] ...
        = getRollCenter(nodesRight, nodesLeft, false, limits);
    
    deltaRoll = roll-prevRoll;
    
    % Update R-UCA
    [parametersRight(1), parametersRight(2)] = rotateCoords( nodesRight(1,:), ...
        [rollResults(i).rollCenterX, rollResults(i).rollCenterY], deltaRoll);
    % Update L-UCA
    [parametersLeft(1), parametersLeft(2)] = rotateCoords( nodesLeft(1,:), ...
        [rollResults(i).rollCenterX, rollResults(i).rollCenterY], deltaRoll);
    % Update R-UCA
    [parametersRight(3), parametersRight(4)] = rotateCoords( nodesRight(2,:), ...
        [rollResults(i).rollCenterX, rollResults(i).rollCenterY], deltaRoll);
    % Update L-UCA
    [parametersLeft(3), parametersLeft(4)] = rotateCoords( nodesLeft(2,:), ...
        [rollResults(i).rollCenterX, rollResults(i).rollCenterY], deltaRoll);
    
    % Incrament counter
    i = i + 1;
    
    % Track angle
    prevRoll = roll;
    
end

% --------------------- Plot Longitudinal Results ---------------------- %

figure;
hold on;
plot([longitudinalResults.disp], [longitudinalResults.camber], 'r-');
ylabel('Camber (degrees)');
yyaxis right
plot([longitudinalResults.disp], [longitudinalResults.lateralScrub], 'b-');
ylabel('Lateral Scrub (mm)');
xlabel('Vertical Chassis Displacement (mm)');
legend('Camber','Lateral Scrub' );
title('Suspension Simulation - Vertical Movement');

% Add parameters
printSimulationParams(parametersRight); 

% Save
saveas(1,'outputs/Suspension Simulation Results - Vertical Movement','png');

% --------------------- Plot Cornering Results ---------------------- %

figure;
hold on;
plot([rollResults.camber], 'r-');
ylabel('Camber (degrees)');
yyaxis right
plot([rollResults.lateralScrub], 'b-');
ylabel('Lateral Scrub (mm)');
xlabel('Roll');
legend('Camber','Lateral Scrub' );
title('Suspension Simulation - Cornering');

% Add parameters
printSimulationParams(parametersRight); 

% Save
saveas(1,'outputs/Suspension Simulation Results - Cornering','png');
