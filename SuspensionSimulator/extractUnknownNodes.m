% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - extractUnknownNodes.m
% A function that returns the unknown nodes in the suspension 
% This function is used to parse the guess. 
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2016
%
%
% Notes:
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
% ---------------------------------------------------------------------- %

function unknowns = extractUnknownNodes(nodes)
    unknowns = [ nodes(3,1);
                 nodes(3,2);
                 nodes(4,1);
                 nodes(4,2);
                 nodes(5,1);
                 nodes(5,2);
                 nodes(6,1);
                 nodes(6,2);
                 nodes(7,1);
                 nodes(7,2);
                 nodes(8,1)  ];
end 