function report = validateStage4Integration(simulationOutput)
%VALIDATESTAGE4INTEGRATION Validate Stage 3 logs before 3-D playback.

requiredLogs = { ...
    'true_state3_log',16; ...
    'estimated_state3_log',12; ...
    'environment3_log',9; ...
    'reference3_log',4};

report = struct;
report.Passed = true;
report.Messages = strings(0,1);

fprintf('\nSTAGE 4 INTEGRATION CHECK\n');
fprintf('=========================\n');

logData = cell(size(requiredLogs,1),1);
logTime = cell(size(requiredLogs,1),1);
for k = 1:size(requiredLogs,1)
    name = requiredLogs{k,1};
    width = requiredLogs{k,2};
    try
        [logTime{k},logData{k}] = readValidationLog( ...
            simulationOutput,name,width);
        fprintf('PASS  %-25s width %d, samples %d\n', ...
            name,width,numel(logTime{k}));
    catch exception
        fprintf('FAIL  %-25s %s\n',name,exception.message);
        report.Passed = false;
        report.Messages(end+1) = string(exception.message); %#ok<AGROW>
    end
end

if ~report.Passed
    error('Stage 4 input validation failed. Correct the missing/invalid logs.');
end

tTrue = logTime{1};
xTrue = logData{1};
tEstimate = logTime{2};
xEstimate = logData{2};

if any(~isfinite(xTrue(:)))
    report.Passed = false;
    report.Messages(end+1) = "True state contains NaN or Inf.";
    fprintf('FAIL  True state contains NaN or Inf.\n');
else
    fprintf('PASS  True state contains only finite values.\n');
end

if any(~isfinite(xEstimate(:)))
    report.Passed = false;
    report.Messages(end+1) = "Estimated state contains NaN or Inf.";
    fprintf('FAIL  Estimated state contains NaN or Inf.\n');
else
    fprintf('PASS  Estimated state contains only finite values.\n');
end

if any(diff(tTrue) < 0) || any(diff(tEstimate) < 0)
    report.Passed = false;
    report.Messages(end+1) = "A log time vector is not monotonic.";
    fprintf('FAIL  Log time is not monotonic.\n');
else
    fprintf('PASS  Log time vectors are monotonic.\n');
end

xEstimateAtTrue = interpolateValidation(tEstimate,xEstimate,tTrue);
positionError = xEstimateAtTrue(:,1:3)-xTrue(:,1:3);
attitudeErrorDegrees = (xEstimateAtTrue(:,7:9)-xTrue(:,7:9))*180/pi;

report.Duration = tTrue(end)-tTrue(1);
report.MaximumAbsolutePosition = max(abs(xTrue(:,1:3)),[],1);
report.MaximumAbsoluteAttitudeDegrees = ...
    max(abs(xTrue(:,7:9))*180/pi,[],1);
report.FinalPositionError = positionError(end,:);
report.FinalAttitudeErrorDegrees = attitudeErrorDegrees(end,:);
report.MaximumMotorSpeed = max(xTrue(:,13:16),[],1);

fprintf('\nDuration: %.3f s\n',report.Duration);
fprintf('Maximum |position| [X Y Z]: %.3f  %.3f  %.3f m\n', ...
    report.MaximumAbsolutePosition);
fprintf('Maximum |attitude| [R P Y]: %.2f  %.2f  %.2f deg\n', ...
    report.MaximumAbsoluteAttitudeDegrees);
fprintf('Final estimate-position error: %.3f  %.3f  %.3f m\n', ...
    report.FinalPositionError);
fprintf('Final estimate-attitude error: %.2f  %.2f  %.2f deg\n', ...
    report.FinalAttitudeErrorDegrees);
fprintf('Maximum motor speeds: %.1f  %.1f  %.1f  %.1f rad/s\n', ...
    report.MaximumMotorSpeed);

if report.Duration < 17.5
    warning('Stage4:ShortRun','Expected approximately 18 seconds of Stage 3 data.');
end
if any(report.MaximumMotorSpeed >= 899.0)
    warning('Stage4:MotorSaturation','At least one motor approached its limit.');
end
if any(report.MaximumAbsoluteAttitudeDegrees(1:2) > 45.0)
    warning('Stage4:LargeTilt','Roll or pitch exceeded 45 degrees.');
end
if norm(report.FinalPositionError) > 0.75
    warning('Stage4:EstimateError','Final position-estimation error is large.');
end

if report.Passed
    fprintf('\nRESULT: PASS - data is ready for Stage 4 visualization.\n\n');
else
    error('Stage 4 input validation failed.');
end
end

function [time,data] = readValidationLog(simulationOutput,name,requiredWidth)
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
if size(data,2) < requiredWidth
    error('%s needs %d columns but has %d.', ...
        name,requiredWidth,size(data,2));
end
data = data(:,1:requiredWidth);
end

function result = interpolateValidation(time,data,newTime)
[uniqueTime,indices] = unique(time,'stable');
result = interp1(uniqueTime,data(indices,:),newTime,'linear','extrap');
end
