close all
clear all

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

zRest = (sprungMass*9.81)/(4*18000); % this is the natural deltaZ position

lf = 1.15; lr = 1.15;
hcg = 0.85; w = 1.3;
wheelRadius = 0.3; 
xDistance = [1.15, 1.15, 1.15, 1.15]; 
yDistance = 1; 
timeMax = 5; IWheel = 7;

%================================ Inputs =================================%
TMotor = [5, 5, 0, 0, 0, 0, 0]; % Motor torque at each time 
TBraking = [0, 0, 0, 0, 0, 0, 0]; % Braking torque at each time
delta = [0, 0, 0, 0, 0, 0, 0]; % Wheel angle at each time 
%delta = [0, 0, 0, 0, 0, 0, 0]; % Wheel angle at each time 
timeStep = 0.5;

for time = 1:7
     if time == 1
        
        for i = 1:4
            FLat(time,i) = 0; Fdrive(time,i) = 0; 
            Fx(time,i) = 0; Fy(time,i) = 0; Fz(time,i) = sprungMass*9.81/4;
            deltaZ(time,i) = zRest; deltaZPrime(time,i) = 0;
        end
        yawAcceleration(time,1) = 0; yawRate(time,1) = 0; yawAngle(time,1) = 0;
        accelerationX(time,1) = 0; accelerationY(time,1) = 0; accelerationZ(time,1) = 0;
        velocityX(time,1) = 1e-8; velocityY(time,1) = 0; velocityZ(time,1) = 0;
        positionX(time,1) = 0; positionY(time,1) = 0; positionZ(time,1) = 0;
        roll(time,1) = 0; pitch(time,1) = 0; rollRate(time,1) = 0; pitchRate(time,1) = 0;
        pitchAcceleration(time,1) = 0; rollAcceleration(time,1) = 0;
        
     else
         for i3 = 1:4
            Fz(time,i3) = K(1,i3)*deltaZ(time-1,i3)+C(1,i3)*deltaZPrime(time-1,i3);
            FLat(time,:) = tireModelY(velocityX(time-1,1),velocityY(time-1,1),lf,yawRate(time-1,1),w,delta(1,time), Fz(time-1,i3));
            %FLong(time,:) = tireModelX(velocityX(time-1,1),velocityY(time-1,1),lf,yawAngle(time-1,1),w,delta(1,time), Fz(time,i3));
            Fdrive(time,:) = torqueModel(TMotor(1,time-1),TBraking(1,time-1),IWheel,accelerationX(time-1,1),wheelRadius);
            Fx(time,:) = - FLat(time,:)*sind(delta(1,time)) + Fdrive(time,:); 
            Fy(time,:) = FLat(time,:)*cosd(delta(1,time)); 
         end 
         
        accelerationZ(time,1) = (sum(Fz(time,:))-9.81*sprungMass)/sprungMass; 
        accelerationX(time,1) = (Fx(time,1)*cosd(delta(1,time))-Fy(time,1)*sind(delta(1,time))+Fx(time,2)*cosd(delta(1,time))-Fy(time,2)*sind(delta(1,time))...
            +Fx(time,3)+Fx(time,4))/sprungMass;
        accelerationY(time,1) = (Fy(time,1)*cosd(delta(1,time))-Fx(time,1)*sind(delta(1,time))+Fy(time,2)*cosd(delta(1,time))-Fx(time,2)*sind(delta(1,time))...
            +Fy(time,3)+Fy(time,4))/sprungMass;
        
%         velocityX(time,1) = velocityX(time-1,1) + accelerationX(time,1)*timeStep;
%         velocityY(time,1) = velocityY(time-1,1) + accelerationY(time,1)*timeStep; 
%         velocityZ(time,1) = velocityZ(time-1,1) + accelerationZ(time,1)*timeStep; 

        velocityX(time,1) = (timeStep)*((accelerationX(time,1)+accelerationX(time-1,1))/2);
        velocityY(time,1) = (timeStep)*((accelerationY(time,1)+accelerationY(time-1,1))/2);
        velocityZ(time,1) = (timeStep)*((accelerationZ(time,1)+accelerationZ(time-1,1))/2);
       
        positionX(time,1) = (timeStep) * ((velocityX(time,1)+velocityX(time-1,1)/2));
        positionY(time,1) = (timeStep) * ((velocityY(time,1)+velocityY(time-1,1)/2));
        positionZ(time,1) = (timeStep) * ((velocityZ(time,1)+velocityZ(time-1,1)/2));
        
        rFL(time,:) = [xDistance(1,1), yDistance, -hcg + deltaZ(time-1,1)];
        rFR(time,:) = [xDistance(1,2), -yDistance, -hcg + deltaZ(time-1,2)];
        rRL(time,:) = [-xDistance(1,3), yDistance, -hcg + deltaZ(time-1,3)];
        rRR(time,:) = [-xDistance(1,4), -yDistance, -hcg + deltaZ(time-1,4)];
        
        F_FL(time,:) = [Fx(time,1), Fy(time,1), Fz(time,1)];
        F_FR(time,:) = [Fx(time,2), Fy(time,2), Fz(time,2)];
        F_RL(time,:) = [Fx(time,3), Fy(time,3), Fz(time,3)];
        F_RR(time,:) = [Fx(time,4), Fy(time,4), Fz(time,4)];
        
        mFL(time,:) = cross(rFL(time,:), F_FL(time,:),2); 
        mFR(time,:) = cross(rFR(time,:), F_FR(time,:),2);
        mRL(time,:) = cross(rRL(time,:), F_RL(time,:),2);
        mRR(time,:) = cross(rRR(time,:), F_RR(time,:),2); 
        
        rollAcceleration(time,1) = (mFL(time,1)+mFR(time,1)+mRL(time,1)+mRR(time,1))/Ixx; 
        pitchAcceleration(time,1) = (mFL(time,2)+mFR(time,2)+mRL(time,2)+mRR(time,2))/Iyy;
        yawAcceleration(time,1) = (mFL(time,3)+mFR(time,3)+mRL(time,3)+mRR(time,3))/Izz; 
        
        pitchRate(time,1) = (timeStep) * ((pitchAcceleration(time,1)+pitchAcceleration(time-1,1))/2); 
        pitch(time,1) = (timeStep) * ((pitchRate(time,1)+pitchRate(time-1,1))/2);
        rollRate(time,1) = (timeStep) * ((rollAcceleration(time,1)+rollAcceleration(time-1,1))/2); 
        roll(time,1) = (timeStep) * ((rollRate(time,1)+rollRate(time-1,1))/2);
        yawRate(time,1) = (timeStep)*((yawAcceleration(time,1) + yawAcceleration(time-1,1))/2);
        yawAngle(time,1) = (timeStep) * ((yawRate(time,1)+yawRate(time-1,1))/2);
        
         for i2 = 1:4
            deltaZFL(time,1) = 0.5*w*sind(roll(time,1))-0.5*lf*sind(pitch(time,1))+zRest; % add cnstant to compensate for uncomp ref position
            deltaZFR(time,1) = -0.5*w*sind(roll(time,1))-0.5*lf*sind(pitch(time,1))+zRest;
            deltaZRL(time,1) = 0.5*w*sind(roll(time,1))+0.5*lf*sind(pitch(time,1))+zRest;
            deltaZRR(time,1) = -0.5*w*sind(roll(time,1))+0.5*lf*sind(pitch(time,1))+zRest;
            deltaZ(time,:) = [deltaZFL(time,1), deltaZFR(time,1), deltaZRL(time,1), deltaZRR(time,1)];
            deltaZPrime(time,i2) = (deltaZ(time,i2)-deltaZ(time-1,i2))/timeStep;
         end 
     end
end 

figure(1)
plot(1:1:7, accelerationX, 1:1:7, accelerationY, 1:1:7, accelerationZ); 

%plot(1:1:7, Fz(:,1), 1:1:7, Fz(:,2), 1:1:7, Fz(:,3), 1:1:7, Fz(:,4)); 
%figure(2)
plot(1:1:7, deltaZ(:,1), 1:1:7, deltaZ(:,2), 1:1:7, deltaZ(:,3), 1:1:7, deltaZ(:,4)); 

figure(3)
plot(1:1:7, TMotor - TBraking); 


