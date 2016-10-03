% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - assembleSolution.m
% A function that returns solved suspension geometry from the output of 
% fsolve and the input parameters
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2016
%
%
% Notes:
%
%    - The order of params is as follows:
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
%    - The order of unknonws is as follows 
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
%    - In nodes, each row represents a node: (x,y)
%
% ---------------------------------------------------------------------- %

function nodes = assembleSolution(solvedNodes, params)
    nodes = [ params(1),       params(2);       % UCA mount
              params(3),       params(4);       % LCA mount
              solvedNodes(1),  solvedNodes(2);  % upright top
              solvedNodes(3),  solvedNodes(4);  % upright spindle mount
              solvedNodes(5),  solvedNodes(6);  % upright btm
              solvedNodes(7),  solvedNodes(8);  % wheel top
              solvedNodes(9),  solvedNodes(10); % wheel center
              solvedNodes(11), 0    ];          % wheel btm 
end 