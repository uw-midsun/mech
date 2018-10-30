% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - computeNodalPositions.m
% Function to compute x,y position of nodes from properties
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2016
%
%
% Notes:
%
%   - The order of params is as follows:
%        1)  UCA mount (x)
%        2)  UCA mount (y)
%        3)  LCA mount (x)
%        4)  LCA mount (y)
%        5)  UCA length
%        6)  LCA length
%        7)  upright length
%        8)  spindle height
%        9)  wheel offset
%        10) wheel diameter
%        11) inclination angle
%
% ---------------------------------------------------------------------- %

function nodes = computeNodalPositions(params, lcaAngle)
    
    % Control arm mounting postions
    nodes(1,1) = params(1);
    nodes(1,2) = params(2);
    nodes(2,1) = params(3);
    nodes(2,2) = params(4);
    
    % 
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % End of LCA
    nodes(5,1) = nodes(2,1) + params(6) * cosd(lcaAngle);
    nodes(5,2) = nodes(2,2) + params(6) * sind(lcaAngle);
    
    % End of UCA 
    [xout,yout] = circcirc(x1,y1,r1,x2,y2,r2)
    nodes(3,1) = nodes(1,1) + ((nodes(1,1) + UCA - y)*(UCA - Ay + y))^(1/2)
    
    % Distance from the UCA mount to the end of the LCA
    ucaMount2LCAend = sqrt((nodes(1,1)-nodes(5,1))^2 ... 
                         + (nodes(1,2)-nodes(5,2))^2); % by distance formula
    
    % Temp angle is the angle formed btwn the UCA and the line from the UCA
    % mount to the end of the LCA
    tempAngle = acosd( (params(5)^2 + ucaMount2LCAend^2 - params(7)^2)...
                      /(2 * params(5) * ucaMount2LCAend)); % by cosine law
    
    % End of UCA 
    
    
end 
