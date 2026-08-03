function omegaCommand = attitudeMotorControllerStage11( ...
    xEstimate,safeAcceleration,yawDesired,controlEnabled)
%ATTITUDEMOTORCONTROLLERSTAGE11 Safe acceleration to motor commands.

%#codegen

m=1.20; g=9.81; arm=0.23; kf=1.8e-5; km=2.5e-7; omegaMax=900.0;
omegaHover=sqrt(m*g/(4.0*kf));
if ~controlEnabled(1)
    omegaCommand=omegaHover*ones(4,1);
    return;
end

roll=xEstimate(7); pitch=xEstimate(8); yaw=xEstimate(9);
bodyRates=zeros(3,1);
bodyRates(1)=xEstimate(10); bodyRates(2)=xEstimate(11); bodyRates(3)=xEstimate(12);

acceleration=min(max(safeAcceleration,[-3.0;-3.0;-5.0]),[3.0;3.0;5.0]);
rollDesired=(sin(yaw)*acceleration(1)-cos(yaw)*acceleration(2))/g;
pitchDesired=(cos(yaw)*acceleration(1)+sin(yaw)*acceleration(2))/g;
maxTilt=25*pi/180;
rollDesired=min(max(rollDesired,-maxTilt),maxTilt);
pitchDesired=min(max(pitchDesired,-maxTilt),maxTilt);
yawError=atan2(sin(yawDesired-yaw),cos(yawDesired-yaw));
angleError=[rollDesired-roll;pitchDesired-pitch;yawError];

torque=[0.80;0.80;0.30].*angleError-[0.18;0.18;0.12].*bodyRates;
torque=min(max(torque,[-0.80;-0.80;-0.20]),[0.80;0.80;0.20]);

tiltFactor=max(cos(roll)*cos(pitch),0.30);
totalThrust=m*(g+acceleration(3))/tiltFactor;
totalThrust=min(max(totalThrust,0.0),2.5*m*g);

S=totalThrust/kf;
a=torque(1)/(arm*kf); b=torque(2)/(arm*kf); c=torque(3)/km;
squared=zeros(4,1);
squared(1)=S/4-b/2+c/4;
squared(2)=S/4+a/2-c/4;
squared(3)=S/4+b/2+c/4;
squared(4)=S/4-a/2-c/4;
squared=min(max(squared,0.0),omegaMax^2);
omegaCommand=sqrt(squared);
end
