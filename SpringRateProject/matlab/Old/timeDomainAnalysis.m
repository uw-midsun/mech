syms t c1 c2 k1 k2 m I a b t0 A wb xddot xdot x thetaddot thetadot theta c k

Ax2 = m;% X terms of ODE (A) 
Ax1 = (c1+c2) ; 
Ax0 = (k1+k2);
Atheta2 = 0;
Atheta1 = (b*c2-a*c1);
Atheta0 = (b*k2-a*k1);
Bx2 = 0;
Bx1 = (b*c2-a*c1); 
Bx0 = (b*k2-a*k1); 
Btheta2 = I;
Btheta1 = (b^2*c2+a^2*c1);
Btheta0 = (b^2*k2+a^2*k1);

PA = c1*A*wb*cos(wb*t) + c2*A*wb*cos(wb*(t-t0)) + k1*A*sin(wb*t) + k2*A*sin(wb*(t-t0)); 
PB = -c1*a*A*wb*cos(wb*t) + c2*b*A*wb*cos(wb*(t-t0)) + k1*a*A*sin(wb*t) + k2*b*A*sin(wb*(t-t0)); 

% Create linear systems in to solve for transfer functions

A = [Ax2, Ax1, Ax0, Atheta2, Atheta1, Atheta0, PA;
     Bx2, Bx1, Bx0, Btheta2, Btheta1, Btheta0, PB];
 
A = subs(A, {b, c1, c2, k1, k2}, {a, c, c, k, k});
A