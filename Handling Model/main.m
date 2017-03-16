close all
clear all

%change
%=============================== Constants ===============================%
% Most of the contants are extracted from Ubaidillah et. al. 2015 
% Constants at Line 73 of main.m and Line 17 of tireModel.m are completely
% made up, as noted. Any suggestions welcome for proper (or even acceptable approximations)
% of these values would be appreciated. 

totalMass = 1600;
sprungMass = 1400; % kgs
Iyy = 1263.5; % kgm^2
Ixx = 450.1;
Izz = 1263.5;
mufl = 40; murl = 35; murr = 35; mufr = 40;
K = [18000, 18000, 18000, 18000]; % K values at each corner (see correspondences below)
C = [1500, 1500, 1500, 1500]; % C values at each corner (see correspondences below)

% Model iterates over each corner (1:4) of the car with the following
% correspondences:
% 1 -> front left
% 2 -> front right
% 3 -> rear left
% 4 -> rear right

lf = 0.95; lr = 0.95;
hcg = 0.85; w = 1.3;
wheelRadius = 0.3; 
timeMax = 5; IWheel = 40;

%================================ Inputs =================================%
TMotor = [0, 10, 2, 3, 0]; % Motor torque at each time 
TBraking = [0, 2, 0, 10, 0]; % Braking torque at each time
delta = [0, 0, 0, 0, 0]; % Wheel angle at each time 
timeStep = 1;

%============================== Solution =================================%
for time = 1:timeStep:timeMax
    
    %======================== Inital Conditions ==========================%
    if time == 1
        
        for i = 1:4
            Fx(time,i) = 0.2; Fy(time,i) = 0; Fz(time,i) = totalMass*9.81;
            accelerationX(time,i) = 0; accelerationY(time,i) = 0; accelerationZ(time,i) = 0;
            velocityX(time,i) = 0.2; velocityY(time,i) = 0; velocityZ(time,i) = 0;
            positionX(time,i) = 0; positionY(time,i) = 0; positionZ(time,i) = 0;
            yawAcceleration(time,1) = 0; yawRate(time,1) = 0; yawAngle(time,1) = 0;
            deltaZ(time,i) = 1e-5; deltaZPrime(time,i) = 0;       
        end
    else
        
        for i2 = 1:4
            accelerationZ(time,i2) = Fz(time-1,i2)/(totalMass/4);
            accelerationX(time,i2) = (Fx(time-1,1)*cosd(delta(1,time-1))-Fy(time-1,1)*sind(delta(1,time-1))+Fx(time-1,2)*cosd(delta(1,time-1))-Fy(time-1,2)*sind(delta(1,time-1))...
                +Fx(time-1,3)+Fx(time-1,4))/totalMass;
            accelerationY(time,i2) = (Fy(time-1,1)*cosd(delta(1,time-1))-Fx(time-1,1)*sind(delta(1,time-1))+Fy(time-1,2)*cosd(delta(1,time-1))-Fx(time-1,2)*sind(delta(1,time-1))...
                +Fy(time-1,3)+Fy(time-1,4))/totalMass;
        end 
        
        % Yaw Calculations
        yawAcceleration(time,1) = (1/Izz)*((w*Fx(time-1,1)*cosd(delta(1,time-1))/2)-(w*Fx(time-1,2)*cosd(delta(1,time-1))/2)+(w*Fx(time-1,3)/2)...
            - (w*Fx(time-1,4)/2)-(w*Fy(time-1,1)*sind(delta(1,time-1)))/2+(w*Fy(time-1,2)*sind(delta(1,time-1))/2)+lr*Fy(time-1,3)-lf*Fy(time-1,4)...
            - lf*Fy(time-1,1)*cosd(delta(1,time-1))-lf*Fy(time-1,2)*cosd(delta(1,time-1))+lf*Fx(time-1,1)*sind(delta(1,time-1))-lf*Fx(time-1,2)*sind(delta(1,time-1)));
        yawRate(time,1) = (timeStep)*((yawAcceleration(time,1) + yawAcceleration(time-1,1))/2);
        yawAngle(time,1) = (timeStep) * ((yawRate(time,1)+yawRate(time-1,1))/2);
        
        for i = 1:4
            velocityX(time,i) = (timeStep)*((accelerationX(time,i)+accelerationX(time-1,i))/2);
            velocityY(time,i) = (timeStep)*((accelerationY(time,i)+accelerationY(time-1,i))/2);
            velocityZ(time,i) = (timeStep)*((accelerationZ(time,i)+accelerationZ(time-1,i))/2);
           
            positionX(time,i) = (timeStep) * ((velocityX(time,i)+velocityX(time-1,i)/2));
            positionY(time,i) = (timeStep) * ((velocityY(time,i)+velocityY(time-1,i)/2));
            positionZ(time,i) = (timeStep) * ((velocityZ(time,i)+velocityZ(time-1,i)/2));
                       
            kPhi = 20; Bphi = 20; kTheta = 20; BTheta = 20; % totally made up. I dont even know what these variables stand for - the paper (Ubaidillah et. al. 2015) is very unclear.
            
            pitch(time,i) = pitchFunction(totalMass, hcg, accelerationY(time,i), kPhi, Bphi, Ixx);
            roll(time,i) = rollFunction(totalMass, hcg, accelerationY(time,i), kTheta, BTheta, Iyy);
            
            deltaZFL(time,1) = -0.5*lf*sind(roll(time,1))-0.5*w*sind(pitch(time,i)); 
            deltaZFR(time,1) = 0.5*lf*sind(roll(time,1))-0.5*w*sind(pitch(time,i));
            deltaZRL(time,1) = -0.5*lf*sind(roll(time,1))+0.5*w*sind(pitch(time,i));
            deltaZRR(time,1) = 0.5*lf*sind(roll(time,1))+0.5*w*sind(pitch(time,i));
            deltaZ(time,:) = [deltaZFL(time,1), deltaZFR(time,1), deltaZRL(time,1), deltaZRR(time,1)];
            deltaZPrime(time,i) = (deltaZ(time,i)-deltaZ(time-1,i))/timeStep;
            
            Fz(time,i) = K(1,i)*deltaZ(time,i)+C(1,i)*deltaZPrime(time,i);
            Fy(time,:) = tireModel(velocityX(time,i),velocityY(time,i),lf,yawAngle(time,1),wheelRadius,delta(1,time), Fz(time,i));
            Fx(time,:) = torqueModel(TMotor(1,time),TBraking(1,time),IWheel,accelerationX(time,i),wheelRadius);

        end
        
    end
end
