function report = validateStage5Integration(simulationOutput)
%VALIDATESTAGE5INTEGRATION Validate waypoint, estimator, and scene inputs.

[tTrue,xTrue] = readStage5ValidationLog( ...
    simulationOutput,'true_state4w_log',16,true);
[tEstimate,xEstimate] = readStage5ValidationLog( ...
    simulationOutput,'estimated_state4w_log',12,true);
[tReference,reference] = readStage5ValidationLog( ...
    simulationOutput,'trajectory4w_log',10,true);
[~,environment] = readStage5ValidationLog( ...
    simulationOutput,'environment4w_log',9,true);
[~,segment] = readStage5ValidationLog( ...
    simulationOutput,'segment4w_log',1,true);
[~,sensorData,sensorAvailable] = readStage5ValidationLog( ...
    simulationOutput,'sensor4w_log',18,false);

if any(~isfinite(xTrue(:))) || any(~isfinite(xEstimate(:))) ...
        || any(~isfinite(reference(:))) || any(~isfinite(environment(:)))
    error('Stage 5 input data contains NaN or Inf.');
end

trueAtReference = interpolateStage5Validation(tTrue,xTrue,tReference,'linear');
estimateAtReference = interpolateStage5Validation( ...
    tEstimate,xEstimate,tReference,'linear');
trackingError = trueAtReference(:,1:3)-reference(:,1:3);
estimationError = estimateAtReference(:,1:3)-trueAtReference(:,1:3);

% The scene obstacles match animateWaypointMissionStage5.m.
centers = [2.00,1.75,0.65; -1.85,1.80,0.50; 1.90,-1.70,0.80];
sizes = [0.70,0.70,1.30; 0.90,0.70,1.00; 0.75,0.85,1.60];
lowerBounds = centers-sizes/2.0;
upperBounds = centers+sizes/2.0;
droneRadius = 0.16;
minimumObstacleClearance = inf;
collisionSamples = 0;
for sample = 1:size(xTrue,1)
    position = xTrue(sample,1:3).';
    for obstacle = 1:size(centers,1)
        lower = lowerBounds(obstacle,:).';
        upper = upperBounds(obstacle,:).';
        outsideDistance = max(max(lower-position,zeros(3,1)),position-upper);
        clearance = norm(outsideDistance)-droneRadius;
        minimumObstacleClearance = min(minimumObstacleClearance,clearance);
        if clearance < 0.0
            collisionSamples = collisionSamples+1;
        end
    end
end

report = struct;
report.Duration = tTrue(end)-tTrue(1);
report.TrackingRms = sqrt(mean(trackingError.^2,1));
report.EstimationRms = sqrt(mean(estimationError.^2,1));
report.MaximumTiltDegrees = max(abs(trueAtReference(:,7:8))*180/pi,[],1);
report.MaximumMotorSpeed = max(xTrue(:,13:16),[],1);
report.MinimumObstacleClearance = minimumObstacleClearance;
report.CollisionSamples = collisionSamples;
report.SensorLogAvailable = sensorAvailable;
report.SegmentRange = [min(segment(:)),max(segment(:))];
report.Passed = true;

fprintf('\nSTAGE 5 COMPLETE 3-D INTEGRATION CHECK\n');
fprintf('======================================\n');
fprintf('Duration: %.2f s\n',report.Duration);
fprintf('Tracking RMS [X Y Z]: %.3f  %.3f  %.3f m\n',report.TrackingRms);
fprintf('Estimation RMS [X Y Z]: %.3f  %.3f  %.3f m\n',report.EstimationRms);
fprintf('Maximum tilt [R P]: %.2f  %.2f deg\n',report.MaximumTiltDegrees);
fprintf('Maximum motor speed: %.1f  %.1f  %.1f  %.1f rad/s\n', ...
    report.MaximumMotorSpeed);
fprintf('Minimum obstacle clearance: %.3f m\n',minimumObstacleClearance);
fprintf('Mission segment range: %.0f to %.0f\n',report.SegmentRange);

if sensorAvailable
    fprintf('Optional sensor4w_log: PASS (%d columns)\n',size(sensorData,2));
else
    fprintf(['Optional sensor4w_log: NOT FOUND - GPS graphics will be hidden.\n', ...
             'The main 3-D animation can still run.\n']);
end

if report.Duration < 29.5
    report.Passed = false;
    warning('Stage5:Duration','Expected approximately 30 seconds of data.');
end
if any(report.TrackingRms > 0.60)
    report.Passed = false;
    warning('Stage5:Tracking','Waypoint RMS tracking error is too large.');
end
if any(report.MaximumMotorSpeed >= 899.0)
    report.Passed = false;
    warning('Stage5:Motors','A motor reached its limit.');
end
if any(report.MaximumTiltDegrees > 45.0)
    report.Passed = false;
    warning('Stage5:Tilt','Roll or pitch exceeded 45 degrees.');
end
if collisionSamples > 0
    report.Passed = false;
    warning('Stage5:Collision','The true trajectory intersects a scene obstacle.');
end

if report.Passed
    fprintf('RESULT: PASS - Stage 5 data and scene are ready.\n\n');
else
    fprintf('RESULT: CHECK REQUIRED - correct warnings before playback.\n\n');
end
end

function [time,data,available] = readStage5ValidationLog( ...
    simulationOutput,name,width,required)
available = false;
time = zeros(0,1);
data = zeros(0,width);
outputNames = simulationOutput.who;
if any(strcmp(outputNames,name))
    signal = simulationOutput.get(name);
elseif evalin('base',['exist(''' name ''',''var'')'])
    signal = evalin('base',name);
else
    if required
        error('Missing required log "%s".',name);
    end
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
    if required
        error('%s has incompatible time/data dimensions.',name);
    end
    time = zeros(0,1);
    data = zeros(0,width);
    return;
end
if size(data,2) < width
    if required
        error('%s needs %d columns but has %d.',name,width,size(data,2));
    end
    time = zeros(0,1);
    data = zeros(0,width);
    return;
end
data = data(:,1:width);
available = true;
end

function result = interpolateStage5Validation(time,data,newTime,method)
[uniqueTime,indices] = unique(time,'stable');
result = interp1(uniqueTime,data(indices,:),newTime,method,'extrap');
end
