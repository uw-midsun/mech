contactPatch = [0.8,0,0];
UCAKingPinAxis = [0.706,0.420,0.0133];
LCAKingPinAxis = [0.740,0.190,0.0068];

fx = [1530,0,0];
fy = [0,1240,0];
fz = [0,0,2700];

upright = UCAKingPinAxis - LCAKingPinAxis;
uprightUnitVector = upright./norm(upright)
momentArm = contactPatch - LCAKingPinAxis;
pointOnKingPinAxis = LCAKingPinAxis + dot(momentArm,uprightUnitVector).*uprightUnitVector;
perpMomentArm = contactPatch - pointOnKingPinAxis;

torqueFx = cross(perpMomentArm,fx);
torqueFy = cross(perpMomentArm,fy);
torqueFz = cross(perpMomentArm,fz);

netKingPinAxisTorque = dot(torqueFx,uprightUnitVector) +...
                       dot(torqueFy,uprightUnitVector) +...
                       dot(torqueFz,uprightUnitVector)

% Visualize geometry:                    
% figure; 
% hold on;
% plot3(0,0,0,'ko');
% plot3([-.8;0.8],[0;0],[0;0],'-b');
% plot3([-.8;0.8],[0;0],[-2.6;-2.6],'-g');
% plot3([0;0],[0;0],[0;-2.6],'-m');
% plot3(contactPatch(1),contactPatch(2),contactPatch(3),'*r');
% plot3(UCAKingPinAxis(1),UCAKingPinAxis(2),UCAKingPinAxis(3),'*b');
% plot3(LCAKingPinAxis(1),LCAKingPinAxis(2),LCAKingPinAxis(3),'*b');
% plot3([LCAKingPinAxis(1);UCAKingPinAxis(1)],...
%       [LCAKingPinAxis(2);UCAKingPinAxis(2)],...
%       [LCAKingPinAxis(3);UCAKingPinAxis(3)],'-b');
