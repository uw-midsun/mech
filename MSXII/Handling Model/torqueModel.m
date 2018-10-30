function Fdrive = torqueModel(TMotor, TBraking, Inertia, a, radius)

angAccel = a/radius; 
%Fy = (Inertia*angAccel + TMotor - TBraking)/radius; 
Fdrive = (Inertia*angAccel - TMotor + TBraking)/radius; 


    