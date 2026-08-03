function [windWorld, externalForceWorld, externalTorqueBody, environmentLog] = ...
    environmentStage3(t)
%ENVIRONMENTSTAGE3 Disturbance sequence delayed for sensor calibration.
%
% 0-2 s: estimator calibration, 2 s: takeoff begins,
% 5-7 s: X wind ramp, 8-9 s: Y gust,
% 10-10.25 s: force push, 11-11.15 s: roll torque.

%#codegen

windWorld = zeros(3,1);
externalForceWorld = zeros(3,1);
externalTorqueBody = zeros(3,1);

if t >= 5.0 && t < 7.0
    phase = (t-5.0)/2.0;
    smoothFraction = 0.5*(1.0-cos(pi*phase));
    windWorld(1) = 2.0*smoothFraction;
elseif t >= 7.0
    windWorld(1) = 2.0;
end

if t >= 8.0 && t < 9.0
    windWorld(2) = 1.5*sin(pi*(t-8.0));
end

if t >= 10.0 && t < 10.25
    externalForceWorld(2) = 4.0;
end

if t >= 11.0 && t < 11.15
    externalTorqueBody(1) = 0.12;
end

environmentLog = [windWorld; externalForceWorld; externalTorqueBody];
end
