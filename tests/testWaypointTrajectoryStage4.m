%% TESTWAYPOINTTRAJECTORYSTAGE4 Validate the trajectory before using Simulink
clear;
clc;
close all;

Ts = 0.005;
time = (0:Ts:30).';
trajectory = zeros(length(time),10);
controlEnabled = false(length(time),1);
segmentNumber = zeros(length(time),1);

for k = 1:length(time)
    [reference,enabled,segment] = waypointTrajectoryStage4(time(k));
    trajectory(k,:) = reference.';
    controlEnabled(k) = enabled;
    segmentNumber(k) = segment;
end

if any(~isfinite(trajectory(:)))
    error('Trajectory contains NaN or Inf.');
end

% Check zero velocity and acceleration at all internal segment boundaries.
boundaries = [2,5,9,13,18,23,28];
for boundary = boundaries
    [reference,~,~] = waypointTrajectoryStage4(boundary);
    if norm(reference(4:6)) > 1e-10 || norm(reference(7:9)) > 1e-10
        error('Velocity or acceleration is not zero at t = %.3f s.',boundary);
    end
end

fprintf('Waypoint trajectory validation: PASS\n');
fprintf('Maximum desired speed: %.3f m/s\n', ...
    max(vecnorm(trajectory(:,4:6),2,2)));
fprintf('Maximum desired acceleration: %.3f m/s^2\n', ...
    max(vecnorm(trajectory(:,7:9),2,2)));
fprintf('Maximum yaw command: %.1f degrees\n', ...
    max(abs(trajectory(:,10)))*180/pi);

figure('Name','Stage 4 waypoint position');
plot(time,trajectory(:,1:3),'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Desired position (m)');
legend('X','Y','Z','Location','best');
title('Quintic waypoint position');

figure('Name','Stage 4 waypoint derivatives');
subplot(2,1,1);
plot(time,trajectory(:,4:6),'LineWidth',1.3);
grid on;
ylabel('Velocity (m/s)');
legend('Vx','Vy','Vz','Location','best');
title('Velocity is continuous and returns to zero at each waypoint');
subplot(2,1,2);
plot(time,trajectory(:,7:9),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('Ax','Ay','Az','Location','best');

figure('Name','Stage 4 waypoint path');
plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3), ...
    'LineWidth',2.0,'Color',[0.1,0.55,0.95]);
hold on;
waypointTimes = [2,5,9,13,18,23,28,30];
waypointPositions = zeros(length(waypointTimes),3);
for k = 1:length(waypointTimes)
    reference = waypointTrajectoryStage4(waypointTimes(k));
    waypointPositions(k,:) = reference(1:3).';
end
plot3(waypointPositions(:,1),waypointPositions(:,2), ...
    waypointPositions(:,3),'o','MarkerSize',8, ...
    'MarkerFaceColor',[1.0,0.65,0.1]);
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Smooth 3-D waypoint path');
legend('Trajectory','Waypoints','Location','best');

figure('Name','Stage 4 yaw and segment');
subplot(2,1,1);
plot(time,trajectory(:,10)*180/pi,'LineWidth',1.4);
grid on;
ylabel('Yaw (degrees)');
title('Smooth shortest-path yaw command');
subplot(2,1,2);
stairs(time,segmentNumber,'LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('Segment');
