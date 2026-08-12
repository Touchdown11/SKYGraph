%% PLOTSTAGE4WAYPOINTS Plot waypoint tracking from Simulink
% First run: out = sim("quadrotor_stage4_waypoints");

if ~exist('out','var')
    error('Run first: out = sim("quadrotor_stage4_waypoints");');
end

[tTrue,xTrue] = readWaypointLog(out,'true_state4w_log',16);
[tEstimate,xEstimate] = readWaypointLog(out,'estimated_state4w_log',12);
[tTrajectory,trajectory] = readWaypointLog(out,'trajectory4w_log',10);
[tMotor,motorCommand] = readWaypointLog(out,'motorcmd4w_log',4);
[tEnvironment,environment] = readWaypointLog(out,'environment4w_log',9);
[tSegment,segment] = readWaypointLog(out,'segment4w_log',1);

trueAtTrajectory = interpolateWaypoint(tTrue,xTrue(:,1:12),tTrajectory);
estimateAtTrajectory = interpolateWaypoint(tEstimate,xEstimate,tTrajectory);
actualMotorAtCommand = interpolateWaypoint(tTrue,xTrue(:,13:16),tMotor);

figure('Name','Stage 4 smooth waypoint path');
plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3),':', ...
    'LineWidth',2.0,'Color',[0.95,0.65,0.12]);
hold on;
plot3(xTrue(:,1),xTrue(:,2),xTrue(:,3), ...
    'LineWidth',1.8,'Color',[0.10,0.62,0.95]);
plot3(xEstimate(:,1),xEstimate(:,2),xEstimate(:,3),'--', ...
    'LineWidth',1.2,'Color',[0.95,0.30,0.15]);
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
legend('Desired trajectory','True path','Estimated path','Location','best');
title('3-D waypoint tracking');

figure('Name','Stage 4 position tracking');
axisNames = {'X','Y','Z'};
for axisIndex = 1:3
    subplot(3,1,axisIndex);
    plot(tTrajectory,trajectory(:,axisIndex),':','LineWidth',1.8);
    hold on;
    plot(tTrue,xTrue(:,axisIndex),'LineWidth',1.3);
    plot(tEstimate,xEstimate(:,axisIndex),'--','LineWidth',1.1);
    grid on;
    ylabel([axisNames{axisIndex},' (m)']);
    if axisIndex == 1
        legend('Desired','True','Estimated','Location','best');
        title('Position tracking');
    end
end
xlabel('Time (s)');

figure('Name','Stage 4 trajectory derivatives');
subplot(2,1,1);
plot(tTrajectory,trajectory(:,4:6),'LineWidth',1.3);
grid on;
ylabel('Desired velocity (m/s)');
legend('Vx','Vy','Vz','Location','best');
title('Smooth feedforward references');
subplot(2,1,2);
plot(tTrajectory,trajectory(:,7:9),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Desired acceleration (m/s^2)');
legend('Ax','Ay','Az','Location','best');

figure('Name','Stage 4 tracking errors');
subplot(3,1,1);
plot(tTrajectory,trueAtTrajectory(:,1:3)-trajectory(:,1:3), ...
    'LineWidth',1.2);
grid on;
ylabel('True - desired (m)');
legend('X','Y','Z','Location','best');
title('Trajectory tracking errors');
subplot(3,1,2);
plot(tTrajectory,estimateAtTrajectory(:,1:3)-trueAtTrajectory(:,1:3), ...
    'LineWidth',1.2);
grid on;
ylabel('Estimate - true (m)');
legend('X','Y','Z','Location','best');
subplot(3,1,3);
plot(tTrajectory,[trajectory(:,10)*180/pi, ...
    trueAtTrajectory(:,9)*180/pi,estimateAtTrajectory(:,9)*180/pi], ...
    'LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('Yaw (degrees)');
legend('Desired','True','Estimated','Location','best');

figure('Name','Stage 4 motors and mission segments');
subplot(2,1,1);
plot(tMotor,motorCommand-actualMotorAtCommand,'LineWidth',1.1);
grid on;
ylabel('Command - actual (rad/s)');
legend('Front','Left','Rear','Right','Location','best');
title('Motor lag during trajectory flight');
subplot(2,1,2);
stairs(tSegment,segment,'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Waypoint segment');

figure('Name','Stage 4 waypoint environment');
subplot(3,1,1);
plot(tEnvironment,environment(:,1:3),'LineWidth',1.2);
grid on;
ylabel('Wind (m/s)');
legend('X','Y','Z','Location','best');
subplot(3,1,2);
plot(tEnvironment,environment(:,4:6),'LineWidth',1.2);
grid on;
ylabel('Force (N)');
legend('X','Y','Z','Location','best');
subplot(3,1,3);
plot(tEnvironment,environment(:,7:9),'LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('Torque (N m)');
legend('Roll','Pitch','Yaw','Location','best');

function [time,data] = readWaypointLog(simulationOutput,name,requiredWidth)
outputNames = simulationOutput.who;
if any(strcmp(outputNames,name))
    signal = simulationOutput.get(name);
elseif evalin('base',['exist(''' name ''',''var'')'])
    signal = evalin('base',name);
else
    available = strjoin(outputNames,', ');
    error('Missing "%s". Available output variables: %s',name,available);
end
time = signal.Time(:);
data = squeeze(signal.Data);
if isvector(data)
    data = data(:);
elseif size(data,1) == numel(time)
    % Already samples-by-signals.
elseif size(data,2) == numel(time)
    data = data.';
else
    error('%s has incompatible time/data dimensions.',name);
end
if size(data,2) < requiredWidth
    error('%s needs %d columns but has %d.', ...
        name,requiredWidth,size(data,2));
end
data = data(:,1:requiredWidth);
end

function result = interpolateWaypoint(time,data,newTime)
[uniqueTime,indices] = unique(time,'stable');
result = interp1(uniqueTime,data(indices,:),newTime,'linear','extrap');
end
