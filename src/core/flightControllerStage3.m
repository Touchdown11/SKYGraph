function omegaCommand = flightControllerStage3( ...
    xEstimate, reference, positionIntegral, controlEnabled)
%FLIGHTCONTROLLERSTAGE3 Controller uses estimated—not true—state.

%#codegen

m = 1.20;
g = 9.81;
arm = 0.23;
kf = 1.8e-5;
km = 2.5e-7;
omegaMax = 900.0;
omegaHover = sqrt(m*g/(4.0*kf));

% During the first two seconds, hold equal hover speed for calibration.
if ~controlEnabled
    omegaCommand = omegaHover*ones(4,1);
    return;
end

position = xEstimate(1:3);
velocity = xEstimate(4:6);
roll = xEstimate(7);
pitch = xEstimate(8);
yaw = xEstimate(9);
bodyRates = xEstimate(10:12);

positionIntegral = min(max(positionIntegral,[-4.0;-4.0;-1.5]), ...
    [4.0;4.0;1.5]);

KpPosition = [1.20;1.20;4.00];
KdPosition = [1.60;1.60;3.00];
KiPosition = [0.35;0.35;0.80];
accelFeedback = KpPosition.*(reference(1:3)-position) ...
    - KdPosition.*velocity ...
    + KiPosition.*positionIntegral;
accelFeedback = min(max(accelFeedback,[-3.0;-3.0;-5.0]), ...
    [3.0;3.0;5.0]);

rollDesired = (sin(yaw)*accelFeedback(1) ...
    -cos(yaw)*accelFeedback(2))/g;
pitchDesired = (cos(yaw)*accelFeedback(1) ...
    +sin(yaw)*accelFeedback(2))/g;
maxTilt = 25.0*pi/180.0;
rollDesired = min(max(rollDesired,-maxTilt),maxTilt);
pitchDesired = min(max(pitchDesired,-maxTilt),maxTilt);

yawError = atan2(sin(reference(4)-yaw),cos(reference(4)-yaw));
angleError = [rollDesired-roll;pitchDesired-pitch;yawError];
KpAttitude = [0.80;0.80;0.30];
KdAttitude = [0.18;0.18;0.12];
torqueCommand = KpAttitude.*angleError-KdAttitude.*bodyRates;
torqueLimit = [0.80;0.80;0.20];
torqueCommand = min(max(torqueCommand,-torqueLimit),torqueLimit);

tiltFactor = cos(roll)*cos(pitch);
if tiltFactor < 0.30
    tiltFactor = 0.30;
end
totalThrust = m*(g+accelFeedback(3))/tiltFactor;
totalThrust = min(max(totalThrust,0.0),2.5*m*g);

S = totalThrust/kf;
a = torqueCommand(1)/(arm*kf);
b = torqueCommand(2)/(arm*kf);
c = torqueCommand(3)/km;
speedSquared = zeros(4,1);
speedSquared(1) = S/4.0-b/2.0+c/4.0;
speedSquared(2) = S/4.0+a/2.0-c/4.0;
speedSquared(3) = S/4.0+b/2.0+c/4.0;
speedSquared(4) = S/4.0-a/2.0-c/4.0;
speedSquared = min(max(speedSquared,0.0),omegaMax*omegaMax);
omegaCommand = sqrt(speedSquared);
end
