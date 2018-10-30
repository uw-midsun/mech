% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - printSuspension.m
% Fuction to print the current state of the suspension to a figure
% 
% Original version by                                                
%   Dr. Haim Waisman, Columbia University, New York, Copyright 2009    
% Modified 2013                                                      
%   Dr. Robert Gracie, University of Waterloo
% Modified 2014
%   Christopher Kohar, University of Waterloo
% Modified 2016
%   Devon Copeland, Midnight Sun, University of Waterloo
%
% Notes:
%
%   - nodes is a matrix with n rows of the format x,y representing n nodes
%   - initialState is a matrix of nodes at the suspensions neutral position
%   - elmntMap maps has m rows of the format node1,node2 representing
%     the m element in the suspension
%
% ---------------------------------------------------------------------- %

function printSuspension(initialState,nodes,elmntMap,limits) 

    numbElmnts = size(elmntMap,1); % number of elements
    xInit = initialState(:,1); % isolate x coordinates
    yInit = initialState(:,2); % isolate y coordinates
    x = nodes(:,1); % isolate x coordinates
    y = nodes(:,2); % isolate y coordinates

    % Define plot limits
    xlimits = limits(1,:); 
    ylimits = limits(2,:); 
    
    % Plot the structure
    for i = 1:numbElmnts

        % Plot initial state of the suspension
        XXI = [xInit(elmntMap(i,1)), xInit(elmntMap(i,2))];
        YYI = [yInit(elmntMap(i,1)), yInit(elmntMap(i,2))];
        line(XXI,YYI,'LineWidth',1);
        hold on;

        % PLOT FINAL DEFORMED STRUCTURE
        XXF = [x(elmntMap(i,1)), x(elmntMap(i,2))];
        YYF = [y(elmntMap(i,1)), y(elmntMap(i,2))];
        line(XXF,YYF,'LineWidth',2,'Color','RED','LineStyle','--');hold on;
        % Initial Node Location Labels
        text(XXI(1),YYI(1),sprintf('%0.5g',elmntMap(i,1)));
        text(XXI(2),YYI(2),sprintf('%0.5g',elmntMap(i,2)));
        % Final Node Location Labels
        % text(XXF(1),YYF(1),sprintf('%0.5g',elmntMap(i,1)));
        % text(XXF(2),YYF(2),sprintf('%0.5g',elmntMap(i,2)));
    end
    
    % King pin axis
    kingPinAxisEquation = polyfit([x(3),x(5)],[y(3),y(5)],1);
    plot(-1*kingPinAxisEquation(2)/kingPinAxisEquation(1),0,'c*');
    plot([xlimits(1)*10,xlimits(2)*10], ...
         polyval(kingPinAxisEquation,[xlimits(1)*10,xlimits(2)*10]),'c-');
    
    % Mounting points
    plot(x(1),y(1),'ko','LineWidth',2);
    plot(x(2),y(2),'ko','LineWidth',2);
    
end
    
