function omegaCommand = flightControllerStage2(x, reference, positionIntegral)
%FLIGHTCONTROLLERSTAGE2 Position PID plus attitude PD controller.
%
% x contains at least the original 12 rigid-body states. Elements 13:16,
% when present, are actual motor speeds.
% reference = [desired X; desired Y; desired Z; desired yaw]
% positionIntegral = limited integral of position error (m*s)

%#codegen

% Parameters shared with the plant.
m = 1.20;
g = 9.81;
arm = 0.23;
kf = 1.8e-5;
km = 2.5e-7;
omegaMax = 900.0;

position = x(1:3);
velocity = x(4:6);
roll  = x(7);
pitch = x(8);
yaw   = x(9);
bodyRates = x(10:12);

positionDesired = reference(1:3);
yawDesired = reference(4);

% Apply the same limits as the Limited Integrator in Simulink.
integralUpper = [4.0; 4.0; 1.5];
integralLower = -integralUpper;
positionIntegral = min(max(positionIntegral, integralLower), integralUpper);

% Position PID gains. The integral term removes steady wind offset.
KpPosition = [1.20; 1.20; 4.00];
KdPosition = [1.60; 1.60; 3.00];
KiPosition = [0.35; 0.35; 0.80];

accelFeedback = KpPosition .* (positionDesired - position) ...
    - KdPosition .* velocity ...
    + KiPosition .* positionIntegral;

% Prevent excessive acceleration and tilt commands.
accelFeedback(1) = min(max(accelFeedback(1), -3.0), 3.0);
accelFeedback(2) = min(max(accelFeedback(2), -3.0), 3.0);
accelFeedback(3) = min(max(accelFeedback(3), -5.0), 5.0);

rollDesired = (sin(yaw)*accelFeedback(1) ...
    - cos(yaw)*accelFeedback(2)) / g;
pitchDesired = (cos(yaw)*accelFeedback(1) ...
    + sin(yaw)*accelFeedback(2)) / g;

maxTilt = 25.0*pi/180.0;
rollDesired = min(max(rollDesired, -maxTilt), maxTilt);
pitchDesired = min(max(pitchDesired, -maxTilt), maxTilt);

yawError = atan2(sin(yawDesired-yaw), cos(yawDesired-yaw));
angleError = [rollDesired-roll; pitchDesired-pitch; yawError];

KpAttitude = [0.80; 0.80; 0.30];
KdAttitude = [0.18; 0.18; 0.12];
torqueCommand = KpAttitude .* angleError - KdAttitude .* bodyRates;

% Limit requested moments before mixing them into motor commands.
torqueLimit = [0.80; 0.80; 0.20];
torqueCommand = min(max(torqueCommand, -torqueLimit), torqueLimit);

% Gravity compensation and total-thrust limiting.
tiltFactor = cos(roll)*cos(pitch);
if tiltFactor < 0.30
    tiltFactor = 0.30;
end
totalThrust = m*(g + accelFeedback(3))/tiltFactor;
totalThrust = min(max(totalThrust, 0.0), 2.5*m*g);

% Quadrotor mixer: front, left, rear, right.
S = totalThrust/kf;
a = torqueCommand(1)/(arm*kf);
b = torqueCommand(2)/(arm*kf);
c = torqueCommand(3)/km;

speedSquared = zeros(4,1);
speedSquared(1) = S/4.0 - b/2.0 + c/4.0;
speedSquared(2) = S/4.0 + a/2.0 - c/4.0;
speedSquared(3) = S/4.0 + b/2.0 + c/4.0;
speedSquared(4) = S/4.0 - a/2.0 - c/4.0;

speedSquared = min(max(speedSquared, 0.0), omegaMax*omegaMax);
omegaCommand = sqrt(speedSquared);
end
