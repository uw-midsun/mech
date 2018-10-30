% ---------------------------------------------------------------------- %
%
% Suspension - computeCamberVSTravel.m
% Scripte to greate a fit a polynomial function to camber vs travel  
% 
% Author: Devon Copeland
% 
% © Midnight Sun 2017
%
%
% Notes:
%   
%   - 
%
% ---------------------------------------------------------------------- %

clc
clear all
close all

angleVSTravel = [-35.95, 1.043
                 -30.21, 1.091
                 -24.90, 1.128
                 -23.02, 1.139
                 -19.79, 1.155
                 -16.47, 1.170
                 -14.65, 1.176
                 -11.15, 1.187
                 -8.52, 1.193
                 -6.64, 1.196
                 -2.92, 1.2
                 -0.69, 1.201
                 2.85, 1.199
                 7.98, 1.193
                 11.28, 1.185
                 15.49, 1.172
                 18.72, 1.16
                 22.28, 1.143
                 24.45, 1.132
                 27.21, 1.115
                 28.46, 1.108];
            
plot(angleVSTravel(:,1),angleVSTravel(:,2));
coeffs = polyfit(angleVSTravel(:,1),angleVSTravel(:,2),3)

x = floor(angleVSTravel(1,1)):1:floor(angleVSTravel(end,1));
hold on;
plot(x,polyval(coeffs,x),'r');
