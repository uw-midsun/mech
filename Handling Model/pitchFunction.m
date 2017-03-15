function phiApprox = pitchFunction(ms, hcg, a, kPhi, Bphi, Ixx)

% Attempted to use first order Runge Kutta - please inform of mistakes. 
% For sure not returning correct values. 
% Differential eqn to solve is Eqn 14 of Ubaidillah et. al. 2015 

q0 = [0; 0]; % inital conditions
h = 0.05; 
t = 0:h:10; 
a11 = 0; a12 = 1; 
a21 = (ms*hcg*9.81-kPhi)/Ixx; a22 = (-Bphi)/Ixx; 
A = [a11, a12; a21, a22]; % A matrix
b11 = 0; b21 = (-ms*hcg*a)/Ixx; 
B = [b11; b21]; % B matrix

qstar(:,1) = q0; 
for i =1:(length(t)-1)
    k1 = A*qstar(:,i)+B*1;
    qstar(:,i+1) = qstar(:,i)+k1*h; 
end 

phiApprox = qstar(1,i); 
