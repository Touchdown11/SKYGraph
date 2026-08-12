function omega = flightController(x, reference)
%FLIGHTCONTROLLER Basic cascaded position/attitude controller.
%
% reference = [desired X; desired Y; desired Z; desired yaw]
% Position units are meters; yaw is radians.
% Output omega contains four motor speeds in rad/s.

%#codegen

% Physical parameters (keep these consistent with quadDynamics.m)
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

% Outer loop: position errors create desired accelerations.
KpPosition = [1.2; 1.2; 4.0];
KdPosition = [1.6; 1.6; 3.0];
accelFeedback = KpPosition .* (positionDesired - position) ...
    + KdPosition .* (0.0 - velocity);

% Limit commands so the beginner model remains in a sensible flight region.
accelFeedback(1) = min(max(accelFeedback(1), -3.0), 3.0);
accelFeedback(2) = min(max(accelFeedback(2), -3.0), 3.0);
accelFeedback(3) = min(max(accelFeedback(3), -5.0), 5.0);

% For small/moderate angles, horizontal acceleration determines desired tilt.
rollDesired = (sin(yaw)*accelFeedback(1) ...
    - cos(yaw)*accelFeedback(2)) / g;
pitchDesired = (cos(yaw)*accelFeedback(1) ...
    + sin(yaw)*accelFeedback(2)) / g;

maxTilt = 25.0*pi/180.0;
rollDesired = min(max(rollDesired, -maxTilt), maxTilt);
pitchDesired = min(max(pitchDesired, -maxTilt), maxTilt);

% Keep yaw error between -pi and +pi.
yawError = atan2(sin(yawDesired - yaw), cos(yawDesired - yaw));
angleError = [rollDesired - roll; pitchDesired - pitch; yawError];

% Inner loop: attitude errors create body torques.
KpAttitude = [0.80; 0.80; 0.30];
KdAttitude = [0.18; 0.18; 0.12];
torqueCommand = KpAttitude .* angleError - KdAttitude .* bodyRates;

% Vertical command plus gravity compensation creates total thrust.
tiltFactor = cos(roll)*cos(pitch);
if tiltFactor < 0.30
    tiltFactor = 0.30;
end
totalThrust = m * (g + accelFeedback(3)) / tiltFactor;
totalThrust = min(max(totalThrust, 0.0), 2.5*m*g);

% Mixer: convert total thrust and body torques into squared motor speeds.
% Rotor order: front, left, rear, right.
S = totalThrust / kf;
a = torqueCommand(1) / (arm*kf);
b = torqueCommand(2) / (arm*kf);
c = torqueCommand(3) / km;

speedSquared = zeros(4,1);
speedSquared(1) = S/4.0 - b/2.0 + c/4.0;
speedSquared(2) = S/4.0 + a/2.0 - c/4.0;
speedSquared(3) = S/4.0 + b/2.0 + c/4.0;
speedSquared(4) = S/4.0 - a/2.0 - c/4.0;

% A motor cannot have negative squared speed or exceed its limit.
speedSquared = min(max(speedSquared, 0.0), omegaMax*omegaMax);
omega = sqrt(speedSquared);
end
