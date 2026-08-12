function animateDroneStage4(simulationOutput,varargin)
%ANIMATEDRONESTAGE4 Custom 3-D playback of the Stage 3 simulation.
%
% Basic use:
%   animateDroneStage4(out)
%
% Optional name/value settings:
%   'PlaybackSpeed'  1.0, 2.0, etc. (default 1.5)
%   'FrameRate'      display frames/second (default 30)
%   'CameraMode'     'fixed', 'chase', or 'top' (default 'fixed')
%   'ShowEstimate'   true or false (default true)
%
% Coordinates are the project convention: X forward, Y left, Z up.

parser = inputParser;
addParameter(parser,'PlaybackSpeed',1.5,@(v)isnumeric(v)&&isscalar(v)&&v>0);
addParameter(parser,'FrameRate',30,@(v)isnumeric(v)&&isscalar(v)&&v>=5);
addParameter(parser,'CameraMode','fixed',@(v)ischar(v)||isstring(v));
addParameter(parser,'ShowEstimate',true,@(v)islogical(v)||isnumeric(v));
parse(parser,varargin{:});
options = parser.Results;
cameraMode = lower(string(options.CameraMode));
if ~any(cameraMode == ["fixed","chase","top"])
    error('CameraMode must be "fixed", "chase", or "top".');
end

[tTrue,xTrue] = readStage4Log(simulationOutput,'true_state3_log',16);
[tEstimate,xEstimate] = readStage4Log( ...
    simulationOutput,'estimated_state3_log',12);
[tEnvironment,environment] = readStage4Log( ...
    simulationOutput,'environment3_log',9);
[tReference,reference] = readStage4Log( ...
    simulationOutput,'reference3_log',4);

xEstimateAtTrueTime = interpolateStage4(tEstimate,xEstimate,tTrue,'linear');
environmentAtTrueTime = interpolateStage4( ...
    tEnvironment,environment,tTrue,'linear');
referenceAtTrueTime = interpolateStage4( ...
    tReference,reference,tTrue,'previous');

if any(~isfinite(xTrue(:))) || any(~isfinite(xEstimateAtTrueTime(:)))
    error('True or estimated state contains NaN or Inf. Run Stage 3 validation first.');
end

% Determine display bounds from the flight and reference, with safe margins.
allPositions = [xTrue(:,1:3);xEstimateAtTrueTime(:,1:3); ...
                referenceAtTrueTime(:,1:3)];
minimumPosition = min(allPositions,[],1);
maximumPosition = max(allPositions,[],1);
minimumPosition(3) = min(minimumPosition(3),0.0);
marginXY = 1.5;
marginZ = 0.8;
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

figureHandle = figure('Name','Stage 4 - Custom Quadrotor 3-D Visualization', ...
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
title(axesHandle,'Stage 3 estimated-state flight shown in a custom 3-D scene', ...
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

trueDrone = createTrueDrone(axesHandle);
estimatedDrone = createEstimatedDrone(axesHandle);
if ~logical(options.ShowEstimate)
    set(estimatedDrone.Root,'Visible','off');
    set(estimatedTrail,'Visible','off');
end

legend(axesHandle,[trueTrail,estimatedTrail,referenceMarker,windArrow], ...
    'TextColor',[0.93,0.95,0.98],'Color',[0.10,0.13,0.18], ...
    'EdgeColor',[0.4,0.45,0.5],'Location','northwest');

statusBox = annotation(figureHandle,'textbox',[0.79,0.57,0.20,0.35], ...
    'String','','Color',[0.93,0.96,1.0], ...
    'BackgroundColor',[0.09,0.12,0.17], ...
    'EdgeColor',[0.35,0.45,0.58],'LineWidth',1.2, ...
    'FontName','Consolas','FontSize',10,'Interpreter','none');
annotation(figureHandle,'textbox',[0.79,0.44,0.20,0.09], ...
    'String',sprintf(['Blue: true pose\nOrange: estimated pose\n', ...
                      'Press Esc to stop playback']), ...
    'Color',[0.85,0.89,0.94],'BackgroundColor',[0.09,0.12,0.17], ...
    'EdgeColor',[0.35,0.45,0.58],'FontSize',10,'FitBoxToText','off');

setappdata(figureHandle,'StopStage4Animation',false);
set(figureHandle,'KeyPressFcn',@stage4KeyPress);

switch cameraMode
    case "fixed"
        view(axesHandle,38,24);
        camtarget(axesHandle,[mean(xLimits),mean(yLimits),mean(zLimits)]);
    case "top"
        view(axesHandle,0,90);
        camup(axesHandle,[0,1,0]);
    case "chase"
        view(axesHandle,38,20);
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
            || getappdata(figureHandle,'StopStage4Animation')
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
    set(windArrow,'XData',positionTrue(1),'YData',positionTrue(2), ...
        'ZData',positionTrue(3)+0.25,'UData',currentWind(1), ...
        'VData',currentWind(2),'WData',currentWind(3));

    estimationError = positionEstimated-positionTrue;
    statusText = sprintf([ ...
        'TIME          %6.2f s\n\n', ...
        'TRUE POSITION\nX %8.3f m\nY %8.3f m\nZ %8.3f m\n\n', ...
        'TRUE ATTITUDE\nR %8.2f deg\nP %8.2f deg\nY %8.2f deg\n\n', ...
        'EST. POS ERROR\nX %8.3f m\nY %8.3f m\nZ %8.3f m\n\n', ...
        'WIND [%4.1f %4.1f %4.1f] m/s'], ...
        tTrue(index),positionTrue(1),positionTrue(2),positionTrue(3), ...
        attitudeTrue(1)*180/pi,attitudeTrue(2)*180/pi, ...
        attitudeTrue(3)*180/pi,estimationError(1), ...
        estimationError(2),estimationError(3), ...
        currentWind(1),currentWind(2),currentWind(3));
    set(statusBox,'String',statusText);

    if cameraMode == "chase"
        yaw = attitudeTrue(3);
        cameraPosition = positionTrue ...
            + [-4.0*cos(yaw);-4.0*sin(yaw);2.0];
        campos(axesHandle,cameraPosition.');
        camtarget(axesHandle,(positionTrue+[0;0;0.15]).');
        camup(axesHandle,[0,0,1]);
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

function [time,data] = readStage4Log(simulationOutput,name,requiredWidth)
outputNames = simulationOutput.who;
if any(strcmp(outputNames,name))
    signal = simulationOutput.get(name);
elseif evalin('base',['exist(''' name ''',''var'')'])
    signal = evalin('base',name);
else
    error('Missing log "%s". Run the working Stage 3 model first.',name);
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

function stage4KeyPress(source,event)
if strcmpi(event.Key,'escape')
    setappdata(source,'StopStage4Animation',true);
end
end
