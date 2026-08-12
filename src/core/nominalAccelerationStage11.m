function [nominalAcceleration,yawDesired] = nominalAccelerationStage11( ...
    xEstimate,trajectoryReference,positionIntegral,controlEnabled)
%NOMINALACCELERATIONSTAGE11 Outer-loop trajectory controller only.
% Output is desired world acceleration excluding gravity.

%#codegen

nominalAcceleration = zeros(3,1);
yawDesired = trajectoryReference(10);
if ~controlEnabled(1)
    return;
end

position = zeros(3,1);
velocity = zeros(3,1);
for axis = 1:3
    position(axis) = xEstimate(axis);
    velocity(axis) = xEstimate(3+axis);
end
positionDesired = trajectoryReference(1:3);
velocityDesired = trajectoryReference(4:6);
accelerationDesired = trajectoryReference(7:9);
positionIntegral = min(max(positionIntegral,[-4.0;-4.0;-1.5]), ...
    [4.0;4.0;1.5]);

Kp = [1.20;1.20;4.00];
Kd = [1.60;1.60;3.00];
Ki = [0.35;0.35;0.80];
nominalAcceleration = accelerationDesired ...
    +Kp.*(positionDesired-position) ...
    +Kd.*(velocityDesired-velocity) ...
    +Ki.*positionIntegral;
nominalAcceleration = min(max(nominalAcceleration,[-3.0;-3.0;-5.0]), ...
    [3.0;3.0;5.0]);
end
