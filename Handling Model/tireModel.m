function [Fy,longSlipRatio,fLambda] = tireModel(vx, vy, lf, r, wheelRadius, deltaf, Fz)

for i = 1:2
    if i == 1
        a(i,1) = atan((vy+lf*r)/vx)-deltaf;
        vt(i,1) = sqrt((vy+lf*r)^2+vx^2);
        vwx(i,1) = vt(i,1)*cos(a(i,1));
    else
        a(i,1) = atan((vy+lf*r)/vx);
        vt(i,1) = sqrt((vy+lf*r)^2+vx^2);
        vwx(i,1) = vt(i,1)*cos(a(i,1));
    end
    
    angularVelocity(i,1) = vwx(i,1)/wheelRadius;
    longSlipRatio(i,1) = (vwx(i,1) - angularVelocity(i,1))/vwx(i,1);
    
    TireStiffness = 5; CorneringStiffness = 5; mu = 0.45; % completely made-up
    
    Lambda(i,1) = (mu*Fz*(1+longSlipRatio(i,1)))/2*((TireStiffness*longSlipRatio(i,1))^2+...
        (CorneringStiffness*tan(a(i,1)))^2)^(1/2);
    
    if Lambda < 1
        fLambda(i,1) = (2-Lambda(i,1))*Lambda(i,1);
    else
        fLambda(i,1) = 1;
    end
    
    Fy_p(i,1) = TireStiffness * (longSlipRatio(i,1)/(1+longSlipRatio(i,1)))*fLambda(i,1);
end
Fy = [Fy_p(1,1),Fy_p(1,1),Fy_p(2,1),Fy_p(2,1)]; 
