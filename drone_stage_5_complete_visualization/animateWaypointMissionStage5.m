function animateWaypointMissionStage5(simulationOutput,varargin)
%ANIMATEWAYPOINTMISSIONSTAGE5 Complete 3-D waypoint-mission playback.
%
% Basic use:
%   animateWaypointMissionStage5(out)
%
% Optional name/value settings:
%   'PlaybackSpeed'  1.0, 2.0, etc. (default 1.5)
%   'FrameRate'      display frames/second (default 30)
%   'CameraMode'     'fixed', 'chase', 'top', or 'cockpit' (default fixed)
%   'ShowEstimate'   true or false (default true)
%   'ShowGPS'        true or false (default true when sensor log exists)
%
% Coordinates are the project convention: X forward, Y left, Z up.

parser = inputParser;
addParameter(parser,'PlaybackSpeed',1.5,@(v)isnumeric(v)&&isscalar(v)&&v>0);
addParameter(parser,'FrameRate',30,@(v)isnumeric(v)&&isscalar(v)&&v>=5);
addParameter(parser,'CameraMode','fixed',@(v)ischar(v)||isstring(v));
addParameter(parser,'ShowEstimate',true,@(v)islogical(v)||isnumeric(v));
addParameter(parser,'ShowGPS',true,@(v)islogical(v)||isnumeric(v));
parse(parser,varargin{:});
options = parser.Results;
cameraMode = lower(string(options.CameraMode));
if ~any(cameraMode == ["fixed","chase","top","cockpit"])
    error('CameraMode must be "fixed", "chase", "top", or "cockpit".');
end

[tTrue,xTrue] = readStage4Log(simulationOutput,'true_state4w_log',16);
[tEstimate,xEstimate] = readStage4Log( ...
    simulationOutput,'estimated_state4w_log',12);
[tEnvironment,environment] = readStage4Log( ...
    simulationOutput,'environment4w_log',9);
[tReference,reference] = readStage4Log( ...
    simulationOutput,'trajectory4w_log',10);
[tSegment,segment] = readStage4Log( ...
    simulationOutput,'segment4w_log',1);
[sensorAvailable,tSensor,sensorData] = tryReadStage5Log( ...
    simulationOutput,'sensor4w_log',18);

xEstimateAtTrueTime = interpolateStage4(tEstimate,xEstimate,tTrue,'linear');
environmentAtTrueTime = interpolateStage4( ...
    tEnvironment,environment,tTrue,'linear');
referenceAtTrueTime = interpolateStage4( ...
    tReference,reference,tTrue,'linear');
segmentAtTrueTime = interpolateStage4(tSegment,segment,tTrue,'previous');
if sensorAvailable
    sensorAtTrueTime = interpolateStage4(tSensor,sensorData,tTrue,'previous');
else
    sensorAtTrueTime = zeros(numel(tTrue),18);
end

if any(~isfinite(xTrue(:))) || any(~isfinite(xEstimateAtTrueTime(:)))
    error(['True or estimated state contains NaN or Inf. ', ...
           'Run Stage 4 waypoint validation first.']);
end

% Determine display bounds from the flight and reference, with safe margins.
allPositions = [xTrue(:,1:3);xEstimateAtTrueTime(:,1:3); ...
                referenceAtTrueTime(:,1:3)];
minimumPosition = min(allPositions,[],1);
maximumPosition = max(allPositions,[],1);
% Include the complete custom scene and safety boundary.
minimumPosition(1:2) = min(minimumPosition(1:2),[-2.8,-2.8]);
maximumPosition(1:2) = max(maximumPosition(1:2),[2.8,2.8]);
maximumPosition(3) = max(maximumPosition(3),2.0);
minimumPosition(3) = min(minimumPosition(3),0.0);
marginXY = 0.4;
marginZ = 0.4;
xLimits = [minimumPosition(1)-marginXY,maximumPosition(1)+marginXY];
yLimits = [minimumPosition(2)-marginXY,maximumPosition(2)+marginXY];
zLimits = [minimumPosition(3)-0.15,maximumPosition(3)+marginZ];
if diff(xLimits) < 4.0
    center = mean(xLimits);
    xLimits = center+[-2.0,2.0];
end
if diff(yLimits) < 4.0
    center = mean(yLimits);
    yLimits = center+[-2.0,2.0];
end
if diff(zLimits) < 2.5
    zLimits(2) = zLimits(1)+2.5;
end

figureHandle = figure('Name','Stage 5 - Complete Waypoint Mission 3-D Visualization', ...
    'NumberTitle','off','Color',[0.055,0.075,0.11], ...
    'Position',[80,80,1280,760]);
axesHandle = axes('Parent',figureHandle,'Position',[0.05,0.08,0.72,0.86]);
hold(axesHandle,'on');
axis(axesHandle,'equal');
axis(axesHandle,'vis3d');
grid(axesHandle,'on');
box(axesHandle,'on');
set(axesHandle,'Color',[0.09,0.12,0.17], ...
    'XColor',[0.80,0.85,0.90],'YColor',[0.80,0.85,0.90], ...
    'ZColor',[0.80,0.85,0.90],'GridColor',[0.45,0.52,0.60], ...
    'GridAlpha',0.28);
xlabel(axesHandle,'World X - forward (m)');
ylabel(axesHandle,'World Y - left (m)');
zlabel(axesHandle,'World Z - up (m)');
title(axesHandle,'Stage 4 waypoint mission integrated with true and estimated states', ...
    'Color',[0.93,0.96,1.0]);
xlim(axesHandle,xLimits);
ylim(axesHandle,yLimits);
zlim(axesHandle,zLimits);

% Ground surface and landing pad.
[xGround,yGround] = meshgrid(linspace(xLimits(1),xLimits(2),20), ...
    linspace(yLimits(1),yLimits(2),20));
zGround = zeros(size(xGround));
surf(axesHandle,xGround,yGround,zGround, ...
    'FaceColor',[0.16,0.22,0.20],'FaceAlpha',0.72, ...
    'EdgeColor',[0.30,0.39,0.36],'EdgeAlpha',0.35);
padAngle = linspace(0,2*pi,80);
patch(axesHandle,0.55*cos(padAngle),0.55*sin(padAngle), ...
    0.008*ones(size(padAngle)),[0.16,0.18,0.21], ...
    'EdgeColor',[0.92,0.92,0.92],'LineWidth',2.0);
plot3(axesHandle,[-0.30,0.30],[0,0],[0.012,0.012], ...
    'Color',[0.95,0.95,0.95],'LineWidth',3);
plot3(axesHandle,[0,0],[-0.30,0.30],[0.012,0.012], ...
    'Color',[0.95,0.95,0.95],'LineWidth',3);

% Safety boundary, buildings, and obstacle definitions used by collision checks.
safetyBounds = [-2.6,2.6;-2.6,2.6;0.0,2.4];
drawSafetyBoundary(axesHandle,safetyBounds);
obstacles = createStage5Obstacles(axesHandle);

% Full desired path and waypoint markers are known after simulation.
desiredPath = plot3(axesHandle,reference(:,1),reference(:,2),reference(:,3), ...
    ':','Color',[1.0,0.82,0.12],'LineWidth',2.2, ...
    'DisplayName','Desired path');
[waypointPositions,waypointNumbers] = extractWaypointMarkers( ...
    reference,segment,tReference,tSegment);
for waypointIndex = 1:size(waypointPositions,1)
    plot3(axesHandle,waypointPositions(waypointIndex,1), ...
        waypointPositions(waypointIndex,2),waypointPositions(waypointIndex,3), ...
        'o','MarkerSize',7,'MarkerFaceColor',[1.0,0.82,0.12], ...
        'MarkerEdgeColor',[0.15,0.12,0.02]);
    text(axesHandle,waypointPositions(waypointIndex,1), ...
        waypointPositions(waypointIndex,2),waypointPositions(waypointIndex,3)+0.10, ...
        sprintf('W%d',waypointNumbers(waypointIndex)), ...
        'Color',[1.0,0.90,0.30],'FontWeight','bold','FontSize',8);
end

% World-axis indicator at the origin.
axisLength = 0.65;
quiver3(axesHandle,0,0,0,axisLength,0,0,0,'Color',[1,0.25,0.2], ...
    'LineWidth',1.8,'MaxHeadSize',0.5);
quiver3(axesHandle,0,0,0,0,axisLength,0,0,'Color',[0.25,1,0.3], ...
    'LineWidth',1.8,'MaxHeadSize',0.5);
quiver3(axesHandle,0,0,0,0,0,axisLength,0,'Color',[0.25,0.55,1], ...
    'LineWidth',1.8,'MaxHeadSize',0.5);

% World-space diagnostics.
trueTrail = plot3(axesHandle,nan,nan,nan,'-','Color',[0.15,0.78,1.0], ...
    'LineWidth',2.0,'DisplayName','True trajectory');
estimatedTrail = plot3(axesHandle,nan,nan,nan,'--', ...
    'Color',[1.0,0.62,0.16],'LineWidth',1.4, ...
    'DisplayName','Estimated trajectory');
referenceMarker = plot3(axesHandle,0,0,0,'p','MarkerSize',14, ...
    'MarkerFaceColor',[1.0,0.88,0.12],'MarkerEdgeColor',[0.2,0.15,0], ...
    'DisplayName','Reference');
windArrow = quiver3(axesHandle,0,0,0,0,0,0,0, ...
    'Color',[0.75,0.45,1.0],'LineWidth',2.2,'MaxHeadSize',0.5, ...
    'DisplayName','Wind velocity');
forceArrow = quiver3(axesHandle,0,0,0,0,0,0,0, ...
    'Color',[1.0,0.25,0.65],'LineWidth',2.2,'MaxHeadSize',0.5, ...
    'DisplayName','External force');
gpsMarker = plot3(axesHandle,nan,nan,nan,'x','MarkerSize',10, ...
    'Color',[0.25,1.0,0.45],'LineWidth',2.0,'DisplayName','GPS');
gpsTrail = plot3(axesHandle,nan,nan,nan,'.','MarkerSize',7, ...
    'Color',[0.25,0.85,0.40],'DisplayName','GPS samples');
if ~sensorAvailable || ~logical(options.ShowGPS)
    set(gpsMarker,'Visible','off');
    set(gpsTrail,'Visible','off');
end

trueDrone = createTrueDrone(axesHandle);
estimatedDrone = createEstimatedDrone(axesHandle);
if ~logical(options.ShowEstimate)
    set(estimatedDrone.Root,'Visible','off');
    set(estimatedTrail,'Visible','off');
end

legend(axesHandle,[desiredPath,trueTrail,estimatedTrail,referenceMarker, ...
    windArrow,forceArrow,gpsMarker], ...
    'TextColor',[0.93,0.95,0.98],'Color',[0.10,0.13,0.18], ...
    'EdgeColor',[0.4,0.45,0.5],'Location','northwest');

statusBox = annotation(figureHandle,'textbox',[0.79,0.48,0.20,0.44], ...
    'String','','Color',[0.93,0.96,1.0], ...
    'BackgroundColor',[0.09,0.12,0.17], ...
    'EdgeColor',[0.35,0.45,0.58],'LineWidth',1.2, ...
    'FontName','Consolas','FontSize',10,'Interpreter','none');
annotation(figureHandle,'textbox',[0.79,0.34,0.20,0.11], ...
    'String',sprintf(['Blue: true pose | Orange: estimate\n', ...
                      'Green: GPS | Gold: desired path\n', ...
                      'Press Esc to stop playback']), ...
    'Color',[0.85,0.89,0.94],'BackgroundColor',[0.09,0.12,0.17], ...
    'EdgeColor',[0.35,0.45,0.58],'FontSize',10,'FitBoxToText','off');

setappdata(figureHandle,'StopStage5Animation',false);
set(figureHandle,'KeyPressFcn',@stage5KeyPress);

switch cameraMode
    case "fixed"
        view(axesHandle,38,24);
        camtarget(axesHandle,[mean(xLimits),mean(yLimits),mean(zLimits)]);
    case "top"
        view(axesHandle,0,90);
        camup(axesHandle,[0,1,0]);
    case "chase"
        view(axesHandle,38,20);
    case "cockpit"
        view(axesHandle,38,12);
end

sampleInterval = median(diff(tTrue));
frameStep = max(1,round(1.0/(options.FrameRate*sampleInterval)));
frameIndices = 1:frameStep:numel(tTrue);
if frameIndices(end) ~= numel(tTrue)
    frameIndices(end+1) = numel(tTrue); %#ok<AGROW>
end

rotorAngle = zeros(4,1);
rotorDirection = [1.0;-1.0;1.0;-1.0];
previousIndex = frameIndices(1);
playbackTimer = tic;
playbackStartTime = tTrue(frameIndices(1));

for frameNumber = 1:numel(frameIndices)
    if ~isgraphics(figureHandle) ...
            || getappdata(figureHandle,'StopStage5Animation')
        break;
    end

    index = frameIndices(frameNumber);
    positionTrue = xTrue(index,1:3).';
    attitudeTrue = xTrue(index,7:9).';
    positionEstimated = xEstimateAtTrueTime(index,1:3).';
    attitudeEstimated = xEstimateAtTrueTime(index,7:9).';

    set(trueDrone.Root,'Matrix',poseMatrix(positionTrue,attitudeTrue));
    set(estimatedDrone.Root,'Matrix', ...
        poseMatrix(positionEstimated,attitudeEstimated));

    deltaTime = tTrue(index)-tTrue(previousIndex);
    rotorAngle = rotorAngle+rotorDirection.*xTrue(index,13:16).'*deltaTime;
    rotorAngle = mod(rotorAngle,2*pi);
    for motor = 1:4
        set(trueDrone.Rotors(motor),'Matrix', ...
            translationMatrix(trueDrone.RotorPositions(:,motor)) ...
            *rotationZMatrix(rotorAngle(motor)));
    end
    previousIndex = index;

    set(trueTrail,'XData',xTrue(1:index,1),'YData',xTrue(1:index,2), ...
        'ZData',xTrue(1:index,3));
    if logical(options.ShowEstimate)
        set(estimatedTrail,'XData',xEstimateAtTrueTime(1:index,1), ...
            'YData',xEstimateAtTrueTime(1:index,2), ...
            'ZData',xEstimateAtTrueTime(1:index,3));
    end

    currentReference = referenceAtTrueTime(index,1:3);
    set(referenceMarker,'XData',currentReference(1), ...
        'YData',currentReference(2),'ZData',currentReference(3));
    currentWind = environmentAtTrueTime(index,1:3);
    currentForce = environmentAtTrueTime(index,4:6);
    set(windArrow,'XData',positionTrue(1),'YData',positionTrue(2), ...
        'ZData',positionTrue(3)+0.25,'UData',currentWind(1), ...
        'VData',currentWind(2),'WData',currentWind(3));
    set(forceArrow,'XData',positionTrue(1),'YData',positionTrue(2), ...
        'ZData',positionTrue(3)+0.12,'UData',0.35*currentForce(1), ...
        'VData',0.35*currentForce(2),'WData',0.35*currentForce(3));

    if sensorAvailable && logical(options.ShowGPS)
        currentGPS = sensorAtTrueTime(index,10:12);
        set(gpsMarker,'XData',currentGPS(1),'YData',currentGPS(2), ...
            'ZData',currentGPS(3));
        gpsIndices = find(sensorAtTrueTime(1:index,17) > 0.5);
        set(gpsTrail,'XData',sensorAtTrueTime(gpsIndices,10), ...
            'YData',sensorAtTrueTime(gpsIndices,11), ...
            'ZData',sensorAtTrueTime(gpsIndices,12));
    end

    estimationError = positionEstimated-positionTrue;
    trackingError = positionTrue-currentReference.';
    currentSegment = round(segmentAtTrueTime(index));
    [collisionDetected,minimumClearance,safetyText] = ...
        checkStage5Safety(positionTrue,obstacles,safetyBounds);
    if collisionDetected
        set(trueDrone.Body,'FaceColor',[0.95,0.10,0.08]);
        statusColor = [1.0,0.35,0.30];
    else
        set(trueDrone.Body,'FaceColor',[0.08,0.48,0.78]);
        statusColor = [0.55,1.0,0.62];
    end

    statusText = sprintf([ ...
        'TIME %6.2f s   SEGMENT %d\nSTATUS %s\nCLEARANCE %6.2f m\n\n', ...
        'TRUE POS [m]\nX %7.3f  Y %7.3f  Z %7.3f\n\n', ...
        'ATTITUDE [deg]\nR %7.2f  P %7.2f  Y %7.2f\n\n', ...
        'TRACK ERROR [m]\nX %7.3f  Y %7.3f  Z %7.3f\n\n', ...
        'EST. ERROR [m]\nX %7.3f  Y %7.3f  Z %7.3f\n\n', ...
        'WIND [%4.1f %4.1f %4.1f] m/s'], ...
        tTrue(index),currentSegment,safetyText,minimumClearance, ...
        positionTrue(1),positionTrue(2),positionTrue(3), ...
        attitudeTrue(1)*180/pi,attitudeTrue(2)*180/pi, ...
        attitudeTrue(3)*180/pi,trackingError(1),trackingError(2), ...
        trackingError(3),estimationError(1),estimationError(2), ...
        estimationError(3),currentWind(1),currentWind(2),currentWind(3));
    set(statusBox,'String',statusText,'Color',statusColor);

    if cameraMode == "chase"
        yaw = attitudeTrue(3);
        cameraPosition = positionTrue ...
            + [-4.0*cos(yaw);-4.0*sin(yaw);2.0];
        campos(axesHandle,cameraPosition.');
        camtarget(axesHandle,(positionTrue+[0;0;0.15]).');
        camup(axesHandle,[0,0,1]);
    elseif cameraMode == "cockpit"
        currentPose = poseMatrix(positionTrue,attitudeTrue);
        forwardDirection = currentPose(1:3,1);
        upDirection = currentPose(1:3,3);
        cameraPosition = positionTrue+0.10*upDirection+0.08*forwardDirection;
        cameraTarget = positionTrue+2.0*forwardDirection+0.08*upDirection;
        campos(axesHandle,cameraPosition.');
        camtarget(axesHandle,cameraTarget.');
        camup(axesHandle,upDirection.');
    end

    drawnow;

    targetWallTime = (tTrue(index)-playbackStartTime)/options.PlaybackSpeed;
    remainingTime = targetWallTime-toc(playbackTimer);
    if remainingTime > 0
        pause(remainingTime);
    end
end
end

function drone = createTrueDrone(axesHandle)
root = hgtransform('Parent',axesHandle);
arm = 0.23;
rotorRadius = 0.095;
rotorPositions = [arm,0,-arm,0;0,arm,0,-arm;0.025,0.025,0.025,0.025];

% Carbon-style arms.
line('Parent',root,'XData',[-arm,arm],'YData',[0,0], ...
    'ZData',[0,0],'Color',[0.12,0.14,0.17],'LineWidth',7);
line('Parent',root,'XData',[0,0],'YData',[-arm,arm], ...
    'ZData',[0,0],'Color',[0.12,0.14,0.17],'LineWidth',7);

% Central body ellipsoid.
[sphereX,sphereY,sphereZ] = sphere(18);
bodySurface = surf(axesHandle,0.105*sphereX,0.075*sphereY, ...
    0.045*sphereZ,'FaceColor',[0.08,0.48,0.78], ...
    'EdgeColor','none','FaceLighting','gouraud');
set(bodySurface,'Parent',root);

% Front marker and body axes.
line('Parent',root,'XData',[0.02,0.18],'YData',[0,0], ...
    'ZData',[0.035,0.035],'Color',[1.0,0.18,0.12],'LineWidth',4);
line('Parent',root,'XData',[0,0.16],'YData',[0,0], ...
    'ZData',[0.07,0.07],'Color',[1,0.25,0.2],'LineWidth',2);
line('Parent',root,'XData',[0,0],'YData',[0,0.16], ...
    'ZData',[0.07,0.07],'Color',[0.25,1,0.3],'LineWidth',2);
line('Parent',root,'XData',[0,0],'YData',[0,0], ...
    'ZData',[0.07,0.23],'Color',[0.25,0.55,1],'LineWidth',2);

circleAngle = linspace(0,2*pi,50);
rotors = gobjects(4,1);
for motor = 1:4
    center = rotorPositions(:,motor);
    line('Parent',root, ...
        'XData',center(1)+rotorRadius*cos(circleAngle), ...
        'YData',center(2)+rotorRadius*sin(circleAngle), ...
        'ZData',center(3)*ones(size(circleAngle)), ...
        'Color',[0.35,0.40,0.46],'LineWidth',1.0);
    rotors(motor) = hgtransform('Parent',root);
    bladeColor = [0.92,0.94,0.97];
    if motor == 1
        bladeColor = [1.0,0.25,0.16];
    end
    line('Parent',rotors(motor),'XData',[-rotorRadius,rotorRadius], ...
        'YData',[0,0],'ZData',[0,0], ...
        'Color',bladeColor,'LineWidth',3.0);
    line('Parent',rotors(motor),'XData',[0,0], ...
        'YData',[-0.55*rotorRadius,0.55*rotorRadius], ...
        'ZData',[0,0],'Color',bladeColor,'LineWidth',1.8);
    line('Parent',rotors(motor),'XData',0,'YData',0,'ZData',0, ...
        'Marker','o','MarkerSize',5,'MarkerFaceColor',[0.08,0.08,0.09], ...
        'MarkerEdgeColor',[0.7,0.7,0.72]);
end

drone.Root = root;
drone.Body = bodySurface;
drone.Rotors = rotors;
drone.RotorPositions = rotorPositions;
end

function drone = createEstimatedDrone(axesHandle)
root = hgtransform('Parent',axesHandle);
arm = 0.23;
rotorRadius = 0.095;
color = [1.0,0.62,0.16];
line('Parent',root,'XData',[-arm,arm],'YData',[0,0], ...
    'ZData',[0,0],'Color',color,'LineStyle','--','LineWidth',2.0);
line('Parent',root,'XData',[0,0],'YData',[-arm,arm], ...
    'ZData',[0,0],'Color',color,'LineStyle','--','LineWidth',2.0);
circleAngle = linspace(0,2*pi,42);
positions = [arm,0,-arm,0;0,arm,0,-arm;zeros(1,4)];
for motor = 1:4
    center = positions(:,motor);
    line('Parent',root, ...
        'XData',center(1)+rotorRadius*cos(circleAngle), ...
        'YData',center(2)+rotorRadius*sin(circleAngle), ...
        'ZData',center(3)*ones(size(circleAngle)), ...
        'Color',color,'LineStyle',':','LineWidth',1.2);
end
line('Parent',root,'XData',[0.02,0.18],'YData',[0,0], ...
    'ZData',[0.02,0.02],'Color',color,'LineStyle','--','LineWidth',2.5);
drone.Root = root;
end

function matrix = poseMatrix(position,euler)
roll = euler(1);
pitch = euler(2);
yaw = euler(3);
cRoll = cos(roll);
sRoll = sin(roll);
cPitch = cos(pitch);
sPitch = sin(pitch);
cYaw = cos(yaw);
sYaw = sin(yaw);
rotation = [ ...
    cYaw*cPitch, cYaw*sPitch*sRoll-sYaw*cRoll, cYaw*sPitch*cRoll+sYaw*sRoll; ...
    sYaw*cPitch, sYaw*sPitch*sRoll+cYaw*cRoll, sYaw*sPitch*cRoll-cYaw*sRoll; ...
    -sPitch,     cPitch*sRoll,                    cPitch*cRoll];
matrix = eye(4);
matrix(1:3,1:3) = rotation;
matrix(1:3,4) = position;
end

function matrix = translationMatrix(position)
matrix = eye(4);
matrix(1:3,4) = position;
end

function matrix = rotationZMatrix(angle)
matrix = [cos(angle),-sin(angle),0,0; ...
          sin(angle), cos(angle),0,0; ...
          0,          0,         1,0; ...
          0,          0,         0,1];
end

function drawSafetyBoundary(axesHandle,bounds)
x = bounds(1,:);
y = bounds(2,:);
z = bounds(3,:);
lineColor = [0.95,0.35,0.20];
for height = [z(1),z(2)]
    plot3(axesHandle,[x(1),x(2),x(2),x(1),x(1)], ...
        [y(1),y(1),y(2),y(2),y(1)],height*ones(1,5), ...
        '--','Color',lineColor,'LineWidth',1.0);
end
for xValue = x
    for yValue = y
        plot3(axesHandle,[xValue,xValue],[yValue,yValue],z, ...
            '--','Color',lineColor,'LineWidth',0.8);
    end
end
end

function obstacles = createStage5Obstacles(axesHandle)
centers = [2.00,1.75,0.65; -1.85,1.80,0.50; 1.90,-1.70,0.80];
sizes = [0.70,0.70,1.30; 0.90,0.70,1.00; 0.75,0.85,1.60];
colors = [0.36,0.43,0.52;0.48,0.38,0.32;0.30,0.45,0.40];
names = ["BUILDING A","BUILDING B","TOWER C"];
for obstacleIndex = 1:size(centers,1)
    drawStage5Box(axesHandle,centers(obstacleIndex,:), ...
        sizes(obstacleIndex,:),colors(obstacleIndex,:));
    text(axesHandle,centers(obstacleIndex,1),centers(obstacleIndex,2), ...
        centers(obstacleIndex,3)+sizes(obstacleIndex,3)/2+0.12, ...
        names(obstacleIndex),'HorizontalAlignment','center', ...
        'Color',[0.86,0.89,0.94],'FontSize',8);
end
obstacles.Centers = centers;
obstacles.Sizes = sizes;
obstacles.Lower = centers-sizes/2.0;
obstacles.Upper = centers+sizes/2.0;
end

function drawStage5Box(axesHandle,center,boxSize,color)
half = boxSize/2.0;
vertices = [ ...
    center(1)-half(1),center(2)-half(2),center(3)-half(3); ...
    center(1)+half(1),center(2)-half(2),center(3)-half(3); ...
    center(1)+half(1),center(2)+half(2),center(3)-half(3); ...
    center(1)-half(1),center(2)+half(2),center(3)-half(3); ...
    center(1)-half(1),center(2)-half(2),center(3)+half(3); ...
    center(1)+half(1),center(2)-half(2),center(3)+half(3); ...
    center(1)+half(1),center(2)+half(2),center(3)+half(3); ...
    center(1)-half(1),center(2)+half(2),center(3)+half(3)];
faces = [1,2,3,4;5,6,7,8;1,2,6,5;2,3,7,6;3,4,8,7;4,1,5,8];
patch(axesHandle,'Vertices',vertices,'Faces',faces, ...
    'FaceColor',color,'FaceAlpha',0.88, ...
    'EdgeColor',[0.72,0.76,0.80],'LineWidth',0.8);
end

function [positions,numbers] = extractWaypointMarkers( ...
    reference,segment,tReference,tSegment)
segmentAtReference = interpolateStage4(tSegment,segment,tReference,'previous');
roundedSegment = round(segmentAtReference);
segmentValues = unique(roundedSegment(roundedSegment >= 1));
positions = reference(1,1:3);
numbers = 0;
for valueIndex = 1:numel(segmentValues)
    value = segmentValues(valueIndex);
    index = find(roundedSegment == value,1,'last');
    positions(end+1,:) = reference(index,1:3); %#ok<AGROW>
    numbers(end+1,1) = value; %#ok<AGROW>
end
end

function [collision,minimumClearance,statusText] = ...
    checkStage5Safety(position,obstacles,safetyBounds)
droneRadius = 0.16;
verticalRadius = 0.07;
positionRow = position.';
clearances = [ ...
    positionRow(1)-safetyBounds(1,1)-droneRadius, ...
    safetyBounds(1,2)-positionRow(1)-droneRadius, ...
    positionRow(2)-safetyBounds(2,1)-droneRadius, ...
    safetyBounds(2,2)-positionRow(2)-droneRadius, ...
    positionRow(3)-safetyBounds(3,1)-verticalRadius, ...
    safetyBounds(3,2)-positionRow(3)-verticalRadius];
onLandingPad = norm(positionRow(1:2)) < 0.50 && positionRow(3) < 0.22;
if onLandingPad
    % Ground contact over the pad is an intended takeoff/landing condition.
    clearances(5) = 0.05;
end
minimumClearance = min(clearances);
collision = minimumClearance < 0.0;
for obstacleIndex = 1:size(obstacles.Lower,1)
    lower = obstacles.Lower(obstacleIndex,:).';
    upper = obstacles.Upper(obstacleIndex,:).';
    outsideDistance = max(max(lower-position,zeros(3,1)),position-upper);
    obstacleClearance = norm(outsideDistance)-droneRadius;
    minimumClearance = min(minimumClearance,obstacleClearance);
    if obstacleClearance < 0.0
        collision = true;
    end
end
if collision
    statusText = 'COLLISION / BOUNDARY';
elseif onLandingPad
    statusText = 'LANDING PAD';
elseif minimumClearance < 0.25
    statusText = 'CAUTION';
else
    statusText = 'SAFE';
end
end

function [available,time,data] = tryReadStage5Log( ...
    simulationOutput,name,requiredWidth)
available = false;
time = zeros(0,1);
data = zeros(0,requiredWidth);
outputNames = simulationOutput.who;
if any(strcmp(outputNames,name))
    signal = simulationOutput.get(name);
elseif evalin('base',['exist(''' name ''',''var'')'])
    signal = evalin('base',name);
else
    return;
end
time = signal.Time(:);
data = squeeze(signal.Data);
if isvector(data)
    data = data(:);
elseif size(data,1) == numel(time)
    % Correct orientation.
elseif size(data,2) == numel(time)
    data = data.';
else
    warning('Stage5:SensorLog','Ignoring sensor log with incompatible dimensions.');
    time = zeros(0,1);
    data = zeros(0,requiredWidth);
    return;
end
if size(data,2) < requiredWidth
    warning('Stage5:SensorLog','Ignoring sensor log with fewer than 18 columns.');
    time = zeros(0,1);
    data = zeros(0,requiredWidth);
    return;
end
data = data(:,1:requiredWidth);
available = true;
end

function [time,data] = readStage4Log(simulationOutput,name,requiredWidth)
outputNames = simulationOutput.who;
if any(strcmp(outputNames,name))
    signal = simulationOutput.get(name);
elseif evalin('base',['exist(''' name ''',''var'')'])
    signal = evalin('base',name);
else
    error('Missing log "%s". Run the working Stage 4 waypoint model first.',name);
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
    error('%s has incompatible Timeseries time/data dimensions.',name);
end
if size(data,2) < requiredWidth
    error('%s requires %d columns but has %d.', ...
        name,requiredWidth,size(data,2));
end
data = data(:,1:requiredWidth);
end

function result = interpolateStage4(time,data,newTime,method)
[uniqueTime,indices] = unique(time,'stable');
result = interp1(uniqueTime,data(indices,:),newTime,method,'extrap');
end

function stage5KeyPress(source,event)
if strcmpi(event.Key,'escape')
    setappdata(source,'StopStage5Animation',true);
end
end
