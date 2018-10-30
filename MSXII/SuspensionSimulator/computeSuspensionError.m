% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - computeSuspensionError.m
% An error function to be minimized by fsolve and solve for nodal positions
% in the process
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2016
%
%
% Notes:
%
%   - parmams is a vector containing the parameters that define the
%     suspension geometry. guess is an vector of unknown nodal coordinates. 
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
%        11) spindle angle
%
%    - The order of the guess is as follows 
%        1)  Node 3 (x)
%        2)  Node 3 (y)
%        3)  Node 4 (x)
%        4)  Node 4 (y)
%        5)  Node 5 (x)
%        6)  Node 5 (y)
%        7)  Node 6 (x)
%        8)  Node 6 (y)
%        9)  Node 7 (x)
%        10) Node 7 (y)
%        11) Node 8 (x)
%
% ---------------------------------------------------------------------- %

function error = computeSuspensionError(guess, params)
    
    % The first six equations are driven by the lengths of the elements 
    % in the suspension (parameters 5 to 10) and are driven by distance
    % formula. 
    error(1) = computeDistance(params(1),params(2),guess(1),guess(2)) ...
               - params(5);  % UCA length
           
    error(2) = computeDistance(params(3),params(4),guess(5),guess(6)) ...
               - params(6);  % LCA length
           
    error(3) = computeDistance(guess(1),guess(2),guess(5),guess(6)) ...
               - params(7);  % upright length
           
    error(4) = computeDistance(guess(3),guess(4),guess(5),guess(6)) ...
               - params(8);  % spindle height
           
    error(5) = computeDistance(guess(3),guess(4),guess(9),guess(10)) ...
               - params(9);  % wheel offset
           
    error(6) = computeDistance(guess(7),guess(8),guess(11),0) ...
               - params(10); % wheel diameter
    
    % The next two equations are driven by the lengths of the two diagonals  
    % drawn from the end of the UCA to the wheel's center and the end of 
    % the LCA and the wheel's center. The equations rely on distance
    % formula and the cosine law. 
    
    error(7) = cosLawGetOppositeLength(params(9),params(7)-params(8),180-params(11)) ...
               - computeDistance(guess(1),guess(2),guess(9),guess(10)); % upper
           
    error(8) = cosLawGetOppositeLength(params(9),params(8),params(11)) ...
               - computeDistance(guess(5),guess(6),guess(9),guess(10)); % lower
    
    % The next two equations are driven by the fact that center of the
    % wheel (node 7) is exactly half way between nodes 6 and 8. 
    
    error(9) = 2*(guess(9)-guess(11)) - (guess(7)-guess(11)); % x
    
    error(10) = 2*guess(10) - guess(8); % y
    
    % The last equation is driven by the fact that the wheel is normal to
    % the spindle and uses the pythagorean theorem: a^2 + b^2 - c^2 = 0
    
    error(11) = computeDistance(guess(3),guess(4),guess(9),guess(10))^2 ...
                + computeDistance(guess(7),guess(8),guess(9),guess(10))^2 ...
                - computeDistance(guess(3),guess(4),guess(7),guess(8))^2;
    
end 

