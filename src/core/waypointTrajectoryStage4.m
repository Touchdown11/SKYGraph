function [trajectoryReference, controlEnabled, segmentNumber] = ...
    waypointTrajectoryStage4(t)
%WAYPOINTTRAJECTORYSTAGE4 Smooth quintic waypoint trajectory.
%
% trajectoryReference layout (10 elements):
%   1:3   desired world position [X;Y;Z] (m)
%   4:6   desired world velocity (m/s)
%   7:9   desired world acceleration (m/s^2)
%   10    desired yaw (rad)
%
% Every segment has zero velocity and acceleration at both ends.

%#codegen

time = t(1);
trajectoryReference = zeros(10,1);
controlEnabled = false;
segmentNumber = 0.0;

position = zeros(3,1);
velocity = zeros(3,1);
acceleration = zeros(3,1);
yaw = 0.0;

if time < 2.0
    % Stationary sensor-calibration period.
    position = [0.0;0.0;0.0];
    controlEnabled = false;
    segmentNumber = 0.0;
elseif time < 5.0
    % Segment 1: smooth vertical takeoff.
    [position,velocity,acceleration,yaw] = quinticSegment( ...
        time,2.0,5.0,[0.0;0.0;0.0],[0.0;0.0;1.0],0.0,0.0);
    controlEnabled = true;
    segmentNumber = 1.0;
elseif time < 9.0
    % Segment 2: move forward and climb slightly.
    [position,velocity,acceleration,yaw] = quinticSegment( ...
        time,5.0,9.0,[0.0;0.0;1.0],[1.0;0.0;1.2],0.0,0.0);
    controlEnabled = true;
    segmentNumber = 2.0;
elseif time < 13.0
    % Segment 3: move left and rotate to 45 degrees.
    [position,velocity,acceleration,yaw] = quinticSegment( ...
        time,9.0,13.0,[1.0;0.0;1.2],[1.0;1.0;1.2],0.0,pi/4.0);
    controlEnabled = true;
    segmentNumber = 3.0;
elseif time < 18.0
    % Segment 4: diagonal movement and yaw to 120 degrees.
    [position,velocity,acceleration,yaw] = quinticSegment( ...
        time,13.0,18.0,[1.0;1.0;1.2],[-0.5;1.0;0.8], ...
        pi/4.0,2.0*pi/3.0);
    controlEnabled = true;
    segmentNumber = 4.0;
elseif time < 23.0
    % Segment 5: return over the landing pad and face forward.
    [position,velocity,acceleration,yaw] = quinticSegment( ...
        time,18.0,23.0,[-0.5;1.0;0.8],[0.0;0.0;1.0], ...
        2.0*pi/3.0,0.0);
    controlEnabled = true;
    segmentNumber = 5.0;
elseif time < 28.0
    % Segment 6: descend to a safe 0.15 m hover above the ground.
    [position,velocity,acceleration,yaw] = quinticSegment( ...
        time,23.0,28.0,[0.0;0.0;1.0],[0.0;0.0;0.15],0.0,0.0);
    controlEnabled = true;
    segmentNumber = 6.0;
else
    % Final hold. The simple plant has no landing-contact model.
    position = [0.0;0.0;0.15];
    velocity = zeros(3,1);
    acceleration = zeros(3,1);
    yaw = 0.0;
    controlEnabled = true;
    segmentNumber = 7.0;
end

trajectoryReference(1:3) = position;
trajectoryReference(4:6) = velocity;
trajectoryReference(7:9) = acceleration;
trajectoryReference(10) = yaw;
end

function [position,velocity,acceleration,yaw] = quinticSegment( ...
    time,startTime,endTime,startPosition,endPosition,startYaw,endYaw)
segmentDuration = endTime-startTime;
normalizedTime = (time-startTime)/segmentDuration;
normalizedTime = min(max(normalizedTime,0.0),1.0);

% Minimum-jerk blend and its first two time derivatives.
blend = 10.0*normalizedTime^3-15.0*normalizedTime^4 ...
        +6.0*normalizedTime^5;
blendRate = (30.0*normalizedTime^2-60.0*normalizedTime^3 ...
            +30.0*normalizedTime^4)/segmentDuration;
blendAcceleration = (60.0*normalizedTime-180.0*normalizedTime^2 ...
                     +120.0*normalizedTime^3)/(segmentDuration^2);

positionChange = endPosition-startPosition;
position = startPosition+positionChange*blend;
velocity = positionChange*blendRate;
acceleration = positionChange*blendAcceleration;

% Follow the shortest angular path between yaw waypoints.
yawChange = atan2(sin(endYaw-startYaw),cos(endYaw-startYaw));
yaw = startYaw+yawChange*blend;
end
