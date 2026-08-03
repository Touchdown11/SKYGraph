function diagnosis = diagnoseStage4WaypointTracking(simulationOutput)
%DIAGNOSESTAGE4WAYPOINTTRACKING Locate causes of excessive RMS error.

[tTrue,xTrue] = readDiagnosticLog(simulationOutput,'true_state4w_log',16);
[tEstimate,xEstimate] = readDiagnosticLog( ...
    simulationOutput,'estimated_state4w_log',12);
[tReference,reference] = readDiagnosticLog( ...
    simulationOutput,'trajectory4w_log',10);
[tSegment,segment] = readDiagnosticLog( ...
    simulationOutput,'segment4w_log',1);

trueAtReference = interpolateDiagnostic(tTrue,xTrue,tReference);
estimateAtReference = interpolateDiagnostic(tEstimate,xEstimate,tReference);
segmentAtReference = interpolatePrevious(tSegment,segment,tReference);
trackingError = trueAtReference(:,1:3)-reference(:,1:3);
estimationError = estimateAtReference(:,1:3)-trueAtReference(:,1:3);

trackingRms = sqrt(mean(trackingError.^2,1));
estimationRms = sqrt(mean(estimationError.^2,1));
maximumTrackingError = zeros(1,3);
peakTime = zeros(1,3);
peakSegment = zeros(1,3);
for axis = 1:3
    [maximumTrackingError(axis),index] = max(abs(trackingError(:,axis)));
    peakTime(axis) = tReference(index);
    peakSegment(axis) = segmentAtReference(index);
end

maximumDesiredSpeed = max(vecnorm(reference(:,4:6),2,2));
maximumDesiredAcceleration = max(vecnorm(reference(:,7:9),2,2));
maximumMotorSpeed = max(xTrue(:,13:16),[],1);
maximumTilt = max(abs(trueAtReference(:,7:8))*180/pi,[],1);

diagnosis = struct;
diagnosis.TrackingRms = trackingRms;
diagnosis.EstimationRms = estimationRms;
diagnosis.MaximumTrackingError = maximumTrackingError;
diagnosis.PeakTime = peakTime;
diagnosis.PeakSegment = peakSegment;
diagnosis.MaximumDesiredSpeed = maximumDesiredSpeed;
diagnosis.MaximumDesiredAcceleration = maximumDesiredAcceleration;
diagnosis.MaximumMotorSpeed = maximumMotorSpeed;
diagnosis.MaximumTiltDegrees = maximumTilt;

fprintf('\nWAYPOINT TRACKING DIAGNOSIS\n');
fprintf('===========================\n');
fprintf('Tracking RMS [X Y Z]:  %.3f  %.3f  %.3f m\n',trackingRms);
fprintf('Estimation RMS [X Y Z]: %.3f  %.3f  %.3f m\n',estimationRms);
fprintf('Peak error [X Y Z]:     %.3f  %.3f  %.3f m\n',maximumTrackingError);
fprintf('Peak-error time:         %.2f  %.2f  %.2f s\n',peakTime);
fprintf('Peak-error segment:      %.0f  %.0f  %.0f\n',peakSegment);
fprintf('Maximum desired speed: %.3f m/s\n',maximumDesiredSpeed);
fprintf('Maximum desired acceleration: %.3f m/s^2\n', ...
    maximumDesiredAcceleration);
fprintf('Maximum tilt [R P]: %.2f  %.2f deg\n',maximumTilt);
fprintf('Maximum motors: %.1f  %.1f  %.1f  %.1f rad/s\n',maximumMotorSpeed);

fprintf('\nAUTOMATIC INTERPRETATION\n');
if maximumDesiredSpeed < 0.05
    fprintf(['FAIL: Desired velocity is nearly zero. trajectory4w_log is ', ...
        'probably connected to the wrong signal.\n']);
elseif maximumDesiredSpeed > 1.5
    fprintf('FAIL: Trajectory is much faster than the supplied mission.\n');
else
    fprintf('PASS: Desired velocity/acceleration appear to be valid.\n');
end

if any(estimationRms > 0.30)
    fprintf(['CHECK ESTIMATOR: estimation RMS is high. Do not increase ', ...
        'controller gains yet.\n']);
elseif any(trackingRms > 0.60)
    fprintf(['CHECK CONTROLLER WIRING: estimator error is moderate but ', ...
        'trajectory tracking is poor. Verify the Stage 4 controller and ', ...
        '10-element reference connection.\n']);
else
    fprintf('PASS: RMS tracking is within the broad validation limit.\n');
end

if any(maximumMotorSpeed >= 899.0)
    fprintf('CHECK SATURATION: at least one motor reached its limit.\n');
else
    fprintf('PASS: motors did not reach the 900 rad/s limit.\n');
end

if any(maximumTilt > 35.0)
    fprintf('CHECK ATTITUDE: excessive tilt indicates control or frame problems.\n');
end

% Inspect the MATLAB Function block scripts when the model is available.
model = "quadrotor_stage4_waypoints";
if bdIsLoaded(model) || ~isempty(which(model+".slx"))
    load_system(model);
    inspectFunctionBlock(model+"/Flight Controller", ...
        'flightControllerStage4Waypoints');
    inspectFunctionBlock(model+"/Waypoint Trajectory", ...
        'waypointTrajectoryStage4');
end
fprintf('\n');
end

function inspectFunctionBlock(blockPath,requiredText)
try
    configuration = get_param(blockPath,'MATLABFunctionConfiguration');
    scriptText = string(configuration.FunctionScript);
    if contains(scriptText,requiredText)
        fprintf('PASS: %s calls %s.\n',blockPath,requiredText);
    else
        fprintf('FAIL: %s does not call %s.\n',blockPath,requiredText);
    end
catch
    fprintf('CHECK MANUALLY: could not inspect %s.\n',blockPath);
end
end

function [time,data] = readDiagnosticLog(simulationOutput,name,width)
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
    error('%s has incompatible dimensions.',name);
end
if size(data,2) < width
    error('%s needs %d columns but has %d.',name,width,size(data,2));
end
data = data(:,1:width);
end

function result = interpolateDiagnostic(time,data,newTime)
[uniqueTime,indices] = unique(time,'stable');
result = interp1(uniqueTime,data(indices,:),newTime,'linear','extrap');
end

function result = interpolatePrevious(time,data,newTime)
[uniqueTime,indices] = unique(time,'stable');
result = interp1(uniqueTime,data(indices,:),newTime,'previous','extrap');
end
