function Fy = torqueModel(TMotor, TBraking, Inertia, a, radius)

angAccel = a/radius; 
Fy = (Inertia*angAccel + TMotor - TBraking)/radius; 

    