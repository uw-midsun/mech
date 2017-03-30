function Fy = tireModelY(vx, vy, lf, r, w, deltaf, Fz)

% for i = 1:2
%     if i == 1
%         a(i,1) = atan2d((vy+lf*r),vx)-deltaf;
%         vt(i,1) = sqrt((vy+lf*r)^2+vx^2);
%         vwx(i,1) = vt(i,1)*cosd(a(i,1));
%     else
%         a(i,1) = atan2d((vy+lf*r),vx);
% 
%     end

      a(1,1) = deltaf - atan2d((vy+lf*r),(vx-(w*r/2))); 
      a(2,1) = deltaf - atan2d((vy+lf*r),(vx+(w*r/2)));
      a(3,1) = -atan2d((vy-lf*r),(vx-(w*r/2)));
      a(4,1) = -atan2d((vy-lf*r),(vx+(w*r/2))); 
      
%       vt(i,1) = sqrt((vy+lf*r)^2+vx^2);
%       vwx(i,1) = vt(i,1)*cosd(a(i,1));
%     wheelRadius = 0.3; 
%     angularVelocity(i,1) = vwx(i,1)/wheelRadius;
%     longSlipRatio(i,1) = (vwx(i,1) - angularVelocity(i,1)*wheelRadius)/vwx(i,1);
%     
    CorneringStiffness = 100; WheelStiffness = 150; mu = 0.45; % completely made-up
    
    
%     a(1,1) = deltaf - atand((vy+lf*r)/(vx-(w*r/2))); 
%     a(2,1) = deltaf - atand((vy+lf*r)/(vx+(w*r/2)));
%     a(3,1) = -atand((vy-lf*r)/(vx-(w*r/2)));
%     a(4,1) = -atand((vy-lf*r)/(vx+(w*r/2))); 
   
for i = 1:4
    Lambda(i,1) = (mu*Fz)/(2*(CorneringStiffness*abs(tand(a(i,1)))));
    %Lambda(i,1) = (mu*Fz+(1+longSlipRatio(i,1)))/(2*sqrt((WheelStiffness*longSlipRatio(i,1))^2 + ...
     %   (CorneringStiffness*tand(a(i,1)))^2));
    
    if Lambda < 1
        fLambda(i,1) = (2-Lambda(i,1))*Lambda(i,1);
    else
        fLambda(i,1) = 1;
    end
    
    %Fy_p(i,1) = CorneringStiffness * (tand(a(i,1))/(1+longSlipRatio(i,1)))*fLambda(i,1);
    Fy_p(i,1) = - CorneringStiffness * tand(a(i,1)) * fLambda(i,1); 
    %Fx_p(i,1) = WheelStiffness * (longSlipRatio(i,1)/(1+longSlipRatio(i,1))) * fLambda(i,1); 
end
Fy = [Fy_p(1,1),Fy_p(2,1),Fy_p(3,1),Fy_p(4,1)]; 