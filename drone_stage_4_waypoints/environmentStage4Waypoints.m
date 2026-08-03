function [windWorld, externalForceWorld, externalTorqueBody, environmentLog] = ...
    environmentStage4Waypoints(t)
%ENVIRONMENTSTAGE4WAYPOINTS Moderate disturbances during waypoint flight.

%#codegen

time = t(1);
windWorld = zeros(3,1);
externalForceWorld = zeros(3,1);
externalTorqueBody = zeros(3,1);
environmentLog = zeros(9,1);

% Smooth X-wind ramp to 1 m/s, then persistent wind.
if time >= 6.0 && time < 8.0
    phase = (time-6.0)/2.0;
    windWorld(1) = 0.5*(1.0-cos(pi*phase));
elseif time >= 8.0
    windWorld(1) = 1.0;
end

% Crosswind gust during the diagonal waypoint segment.
if time >= 14.0 && time < 15.0
    windWorld(2) = 0.8*sin(pi*(time-14.0));
end

% Short lateral force while returning toward the landing pad.
if time >= 20.0 && time < 20.30
    externalForceWorld(2) = 2.0;
end

% Small roll torque during descent.
if time >= 24.0 && time < 24.15
    externalTorqueBody(1) = 0.06;
end

environmentLog(1:3) = windWorld;
environmentLog(4:6) = externalForceWorld;
environmentLog(7:9) = externalTorqueBody;
end
