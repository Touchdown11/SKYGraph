function [windWorld, externalForceWorld, externalTorqueBody, environmentLog] = ...
    environmentStage2(t)
%ENVIRONMENTSTAGE2 Repeatable wind and disturbance test sequence.
%
% Timeline:
%   0-3 s      still air
%   3-5 s      smooth wind ramp to +2 m/s in world X
%   5 s onward constant +2 m/s X wind
%   6-7 s      additional smooth crosswind gust in world Y
%   8-8.25 s   4 N lateral push in world Y
%   9-9.15 s   0.12 N*m roll-torque disturbance

%#codegen

windWorld = zeros(3,1);
externalForceWorld = zeros(3,1);
externalTorqueBody = zeros(3,1);

% Smooth half-cosine wind ramp avoids an impossible instantaneous air change.
if t >= 3.0 && t < 5.0
    phase = (t-3.0)/2.0;
    smoothFraction = 0.5*(1.0-cos(pi*phase));
    windWorld(1) = 2.0*smoothFraction;
elseif t >= 5.0
    windWorld(1) = 2.0;
end

% One-second crosswind gust with zero value at both endpoints.
if t >= 6.0 && t < 7.0
    windWorld(2) = 1.5*sin(pi*(t-6.0));
end

% Short force and torque disturbances.
if t >= 8.0 && t < 8.25
    externalForceWorld(2) = 4.0;
end
if t >= 9.0 && t < 9.15
    externalTorqueBody(1) = 0.12;
end

environmentLog = [windWorld; externalForceWorld; externalTorqueBody];
end
