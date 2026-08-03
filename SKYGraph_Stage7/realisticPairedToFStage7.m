function [distanceM,valid,status,directionDistance,directionValid, ...
    nearestSensorId,deliveredSequence,packetNew,packetAge,dropCount, ...
    idealDistance,faultFlags] = ...
    realisticPairedToFStage7(xTrue,t,ambientMode,faultMode)
%REALISTICPAIREDTOFSTAGE7 Noisy ToF plus delayed/lossy packet channel.
%
% Execute discretely at 0.05 s. The function calls the ideal Stage 6
% geometry, applies offsets/noise/dropouts/faults, and delivers frames with
% one-sample latency. Receiver outputs hold their previous values when a
% packet is lost; packetAge tells consumers when held data is stale.
%
% ambientMode: 0 indoor/normal, 1 bright/low-reflectivity stress
% faultMode:   0 normal
%              1 increased random sensor dropout
%              2 H2 disconnected
%              3 H0 stuck at 0.45 m
%              4 30 percent random packet loss
%              5 communication freeze after 5 s
%
% status: 0 valid, 1 no target, 2 too close, 3 sensor dropout,
%         4 disconnected, 5 stuck value

%#codegen

persistent initialized randomState generatedSequence totalDropCount ...
    pendingDistance pendingValid pendingStatus pendingIdeal ...
    pendingSequence pendingTimestamp pendingFaultFlags pendingAvailable ...
    receiverDistance receiverValid receiverStatus receiverIdeal ...
    receiverSequence receiverTimestamp receiverFaultFlags

maximumRange = 2.0;
time = t(1);
ambient = round(ambientMode(1));
fault = round(faultMode(1));

if isempty(initialized)
    randomState = 246813579.0;
    generatedSequence = 0.0;
    totalDropCount = 0.0;
    pendingDistance = maximumRange*ones(10,1);
    pendingValid = false(10,1);
    pendingStatus = ones(10,1);
    pendingIdeal = maximumRange*ones(10,1);
    pendingSequence = 0.0;
    pendingTimestamp = -1.0e6;
    pendingFaultFlags = 0.0;
    pendingAvailable = false;
    receiverDistance = maximumRange*ones(10,1);
    receiverValid = false(10,1);
    receiverStatus = ones(10,1);
    receiverIdeal = maximumRange*ones(10,1);
    receiverSequence = 0.0;
    receiverTimestamp = -1.0e6;
    receiverFaultFlags = 0.0;
    initialized = true;
end

% Deliver the previous frame first, creating one 0.05 s latency period.
packetNew = false;
if pendingAvailable
    receiverDistance = pendingDistance;
    receiverValid = pendingValid;
    receiverStatus = pendingStatus;
    receiverIdeal = pendingIdeal;
    receiverSequence = pendingSequence;
    receiverTimestamp = pendingTimestamp;
    receiverFaultFlags = pendingFaultFlags;
    packetNew = true;
end
pendingAvailable = false;

% Generate the current ideal geometric frame.
[idealCurrent,idealValid,idealStatus,~,~,~,~,~] = ...
    pairedToFArrayStage6(xTrue,time);
generatedSequence = generatedSequence+1.0;

% Per-sensor deterministic calibration offsets in metres.
offset = [0.006;-0.004;0.003;-0.007;0.005;-0.002;0.008;-0.005;0.004;-0.006];
newDistance = maximumRange*ones(10,1);
newValid = false(10,1);
newStatus = idealStatus;
currentFaultFlags = 0.0;

for sensor = 1:10
    if idealValid(sensor)
        if ambient == 1
            noiseStd = 0.012+0.020*idealCurrent(sensor);
            dropoutProbability = 0.12;
        else
            noiseStd = 0.004+0.006*idealCurrent(sensor);
            dropoutProbability = 0.01;
        end
        if fault == 1
            dropoutProbability = max(dropoutProbability,0.25);
        end

        [uniformSample,randomState] = nextUniformStage7(randomState);
        if uniformSample < dropoutProbability
            newDistance(sensor) = maximumRange;
            newValid(sensor) = false;
            newStatus(sensor) = 3.0;
            currentFaultFlags = 1.0;
        else
            [normalSample,randomState] = nextGaussianStage7(randomState);
            measured = idealCurrent(sensor)+offset(sensor)+noiseStd*normalSample;
            measured = min(max(measured,0.03),maximumRange);
            measured = round(measured*1000.0)/1000.0; % 1 mm quantization
            newDistance(sensor) = measured;
            newValid(sensor) = true;
            newStatus(sensor) = idealStatus(sensor);
        end
    else
        newDistance(sensor) = maximumRange;
        newValid(sensor) = false;
        newStatus(sensor) = idealStatus(sensor);
    end
end

% Deterministic injected sensor failures.
if fault == 2
    newDistance(3) = maximumRange; % zero-based H2 is MATLAB element 3
    newValid(3) = false;
    newStatus(3) = 4.0;
    currentFaultFlags = currentFaultFlags+2.0;
elseif fault == 3
    newDistance(1) = 0.450;
    newValid(1) = true;
    newStatus(1) = 5.0;
    currentFaultFlags = currentFaultFlags+4.0;
end

% Simulated packet channel.
packetDropProbability = 0.0;
if fault == 4
    packetDropProbability = 0.30;
elseif fault == 5 && time >= 5.0
    packetDropProbability = 1.0;
end
[packetRandom,randomState] = nextUniformStage7(randomState);
packetWasDropped = packetRandom < packetDropProbability;
if packetWasDropped
    totalDropCount = totalDropCount+1.0;
    currentFaultFlags = currentFaultFlags+8.0;
else
    pendingDistance = newDistance;
    pendingValid = newValid;
    pendingStatus = newStatus;
    pendingIdeal = idealCurrent;
    pendingSequence = generatedSequence;
    pendingTimestamp = time;
    pendingFaultFlags = double(currentFaultFlags);
    pendingAvailable = true;
end

% Receiver-facing held outputs.
distanceM = receiverDistance;
valid = receiverValid;
status = receiverStatus;
idealDistance = receiverIdeal;
deliveredSequence = receiverSequence;
faultFlags = receiverFaultFlags;
dropCount = totalDropCount;
packetAge = max(time-receiverTimestamp,0.0);

% Coarse six-direction reduction from receiver-held sensor data.
directionDistance = maximumRange*ones(6,1);
directionValid = false(6,1);
nearestSensorId = -ones(6,1);
pairs = [1,2;3,4;5,6;7,8;9,9;10,10];
for direction = 1:6
    for sensor = pairs(direction,1):pairs(direction,2)
        if valid(sensor) && distanceM(sensor) < directionDistance(direction)
            directionDistance(direction) = distanceM(sensor);
            directionValid(direction) = true;
            nearestSensorId(direction) = sensor-1;
        end
    end
end
end

function [sample,state] = nextUniformStage7(state)
modulus = 4294967296.0;
state = mod(1664525.0*state+1013904223.0,modulus);
sample = (state+1.0)/(modulus+1.0);
end

function [sample,state] = nextGaussianStage7(state)
[u1,state] = nextUniformStage7(state);
[u2,state] = nextUniformStage7(state);
u1 = max(u1,1.0e-12);
sample = sqrt(-2.0*log(u1))*cos(2.0*pi*u2);
end
