function report = validateStage4Waypoints(simulationOutput)
%VALIDATESTAGE4WAYPOINTS Check trajectory continuity and closed-loop tracking.

[tTrue,xTrue] = readValidationLog4W(simulationOutput,'true_state4w_log',16);
[tEstimate,xEstimate] = readValidationLog4W( ...
    simulationOutput,'estimated_state4w_log',12);
[tTrajectory,trajectory] = readValidationLog4W( ...
    simulationOutput,'trajectory4w_log',10);

if any(~isfinite(xTrue(:))) || any(~isfinite(xEstimate(:))) ...
        || any(~isfinite(trajectory(:)))
    error('Stage 4 waypoint data contains NaN or Inf.');
end

trueAtTrajectory = interpolateValidation4W(tTrue,xTrue,tTrajectory);
estimateAtTrajectory = interpolateValidation4W(tEstimate,xEstimate,tTrajectory);
trackingError = trueAtTrajectory(:,1:3)-trajectory(:,1:3);
estimationError = estimateAtTrajectory(:,1:3)-trueAtTrajectory(:,1:3);

report = struct;
report.Duration = tTrue(end)-tTrue(1);
report.TrackingRms = sqrt(mean(trackingError.^2,1));
report.MaximumTrackingError = max(abs(trackingError),[],1);
report.EstimationRms = sqrt(mean(estimationError.^2,1));
report.MaximumTiltDegrees = max(abs(trueAtTrajectory(:,7:8))*180/pi,[],1);
report.MaximumMotorSpeed = max(xTrue(:,13:16),[],1);
report.Passed = true;

fprintf('\nSTAGE 4 WAYPOINT VALIDATION\n');
fprintf('===========================\n');
fprintf('Duration: %.2f s\n',report.Duration);
fprintf('Tracking RMS [X Y Z]: %.3f  %.3f  %.3f m\n',report.TrackingRms);
fprintf('Maximum tracking error: %.3f  %.3f  %.3f m\n', ...
    report.MaximumTrackingError);
fprintf('Estimation RMS [X Y Z]: %.3f  %.3f  %.3f m\n',report.EstimationRms);
fprintf('Maximum tilt [roll pitch]: %.2f  %.2f deg\n',report.MaximumTiltDegrees);
fprintf('Maximum motor speeds: %.1f  %.1f  %.1f  %.1f rad/s\n', ...
    report.MaximumMotorSpeed);

if report.Duration < 29.5
    warning('Stage4Waypoints:ShortRun','Expected approximately 30 seconds.');
end
if any(report.MaximumMotorSpeed >= 899.0)
    report.Passed = false;
    warning('Stage4Waypoints:MotorSaturation','A motor reached its limit.');
end
if any(report.MaximumTiltDegrees > 45.0)
    report.Passed = false;
    warning('Stage4Waypoints:LargeTilt','Roll or pitch exceeded 45 degrees.');
end
if any(report.TrackingRms > 0.60)
    report.Passed = false;
    warning('Stage4Waypoints:Tracking','RMS tracking error is too large.');
end

if report.Passed
    fprintf('RESULT: PASS - smooth waypoint tracking is bounded.\n\n');
else
    fprintf('RESULT: CHECK REQUIRED - inspect warnings and plots.\n\n');
end
end

function [time,data] = readValidationLog4W(simulationOutput,name,width)
outputNames = simulationOutput.who;
if any(strcmp(outputNames,name))
    signal = simulationOutput.get(name);
elseif evalin('base',['exist(''' name ''',''var'')'])
    signal = evalin('base',name);
else
    error('Missing log "%s".',name);
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
    error('%s has incompatible time/data dimensions.',name);
end
if size(data,2) < width
    error('%s needs %d columns but has %d.',name,width,size(data,2));
end
data = data(:,1:width);
end

function result = interpolateValidation4W(time,data,newTime)
[uniqueTime,indices] = unique(time,'stable');
result = interp1(uniqueTime,data(indices,:),newTime,'linear','extrap');
end
