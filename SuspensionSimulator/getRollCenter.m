% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - getRollCenter.m
% A function that computes the roll center of the vehicle 
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2016
%
%
% Notes:
%
%   - 
% 
% ---------------------------------------------------------------------- %

function [x,y] = getRollCenter(nodesRight, nodesLeft, PLOT_ROLL_CENTER, limits)
    
    % Right instantaneous center
    rUCAEqn = polyfit([nodesRight(1,1),nodesRight(3,1)], ...
                      [nodesRight(1,2),nodesRight(3,2)],1);
    rLCAEqn = polyfit([nodesRight(2,1),nodesRight(5,1)], ...
                      [nodesRight(2,2),nodesRight(5,2)],1);
    A = [-1 * rUCAEqn(1), 1; -1 * rLCAEqn(1), 1];
    b = [rUCAEqn(2); rLCAEqn(2)];
    rightInstCenter = linsolve(A,b)';
    
    % Left instantaneous center 
    lUCAEqn = polyfit([nodesLeft(1,1),nodesLeft(3,1)], ...
                      [nodesLeft(1,2),nodesLeft(3,2)],1);
    lLCAEqn = polyfit([nodesLeft(2,1),nodesLeft(5,1)], ...
                      [nodesLeft(2,2),nodesLeft(5,2)],1);
    A = [-1 * lUCAEqn(1), 1; -1 * lLCAEqn(1), 1];
    b = [lUCAEqn(2); lLCAEqn(2)];
    leftInstCenter = linsolve(A,b)';
    
    % Roll center
    rightInst2Wheel = polyfit([rightInstCenter(1),nodesRight(8,1)], ...
                              [rightInstCenter(2),0],1);
    leftInst2Wheel = polyfit([leftInstCenter(1),nodesLeft(8,1)], ...
                             [leftInstCenter(2),0],1);
    A = [-1 * rightInst2Wheel(1), 1; -1 * leftInst2Wheel(1), 1];
    b = [rightInst2Wheel(2); leftInst2Wheel(2)];
    rollCenter = linsolve(A,b)';
    x = rollCenter(1);
    y = rollCenter(2);
    
    % Plot roll center geometry
    if PLOT_ROLL_CENTER
        
        extrms = limits(1,:).*100;
        figure(2);
        
        % Axes
        plot(extrms, polyval(rUCAEqn, extrms), 'b-');
        plot(extrms, polyval(lUCAEqn, extrms), 'c-');
        plot(extrms, polyval(rLCAEqn, extrms), 'b-');
        plot(extrms, polyval(lLCAEqn, extrms), 'c-');
        plot(extrms, polyval(rightInst2Wheel, extrms), 'g-');
        plot(extrms, polyval(leftInst2Wheel, extrms), 'g-');
   
        % Points
        plot(rightInstCenter(1),rightInstCenter(2),'b*');
        plot(leftInstCenter(1),leftInstCenter(2),'c*');
        plot(x,y,'g*');
        
    end
    
end 