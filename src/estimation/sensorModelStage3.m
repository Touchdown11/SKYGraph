function [measurement, gpsNew, barometerNew, sensorLog] = ...
    sensorModelStage3(xTrue, dxTrue)
%SENSORMODELSTAGE3 Noisy, biased, multi-rate drone sensors.
%
% This function must execute discretely at Ts = 0.005 s.
%
% measurement layout (16 elements):
%   1:3   gyroscope (rad/s), 200 Hz
%   4:6   accelerometer specific force (m/s^2), 200 Hz
%   7:9   normalized-scale magnetometer, 200 Hz
%   10:12 GPS position (m), held between 5 Hz updates
%   13:15 GPS velocity (m/s), held between 5 Hz updates
%   16    barometer altitude (m), held between 20 Hz updates

%#codegen

persistent sampleCounter randomState gpsPositionHold gpsVelocityHold ...
    barometerHold initialized

if isempty(initialized)
    sampleCounter = 0.0;
    randomState = 123456789.0;
    gpsPositionHold = zeros(3,1);
    gpsVelocityHold = zeros(3,1);
    barometerHold = 0.0;
    initialized = true;
end

% Fixed but unknown sensor biases.
gyroscopeBias = [0.0040; -0.0030; 0.0025];
accelerometerBias = [0.040; -0.030; 0.060];
magnetometerBias = [0.010; -0.008; 0.006];

% White-noise standard deviations.
gyroscopeNoiseStd = 0.0020;
accelerometerNoiseStd = 0.040;
magnetometerNoiseStd = 0.005;
gpsPositionNoiseStd = [0.15; 0.15; 0.25];
gpsVelocityNoiseStd = 0.05;
barometerNoiseStd = 0.05;

g = 9.81;
RbodyToWorld = eulerToRotation(xTrue(7),xTrue(8),xTrue(9));

% Ideal measurements.
gyroscopeIdeal = xTrue(10:12);
% Accelerometer measures specific force, not world acceleration.
accelerometerIdeal = RbodyToWorld.' * (dxTrue(4:6) + [0.0;0.0;g]);
magneticFieldWorld = [0.45;0.0;0.20];
magnetometerIdeal = RbodyToWorld.' * magneticFieldWorld;

% Generate high-rate sensor noise.
gyroscopeNoise = zeros(3,1);
accelerometerNoise = zeros(3,1);
magnetometerNoise = zeros(3,1);
for axis = 1:3
    [gyroscopeNoise(axis),randomState] = nextGaussian(randomState);
    [accelerometerNoise(axis),randomState] = nextGaussian(randomState);
    [magnetometerNoise(axis),randomState] = nextGaussian(randomState);
end

gyroscope = gyroscopeIdeal + gyroscopeBias ...
            + gyroscopeNoiseStd*gyroscopeNoise;
accelerometer = accelerometerIdeal + accelerometerBias ...
                + accelerometerNoiseStd*accelerometerNoise;
magnetometer = magnetometerIdeal + magnetometerBias ...
               + magnetometerNoiseStd*magnetometerNoise;

% GPS: 5 Hz at a 200 Hz base rate, with zero-order hold between updates.
gpsNew = mod(sampleCounter,40.0) == 0.0;
if gpsNew
    gpsPositionNoise = zeros(3,1);
    gpsVelocityNoise = zeros(3,1);
    for axis = 1:3
        [gpsPositionNoise(axis),randomState] = nextGaussian(randomState);
        [gpsVelocityNoise(axis),randomState] = nextGaussian(randomState);
    end
    gpsPositionHold = xTrue(1:3) ...
        + gpsPositionNoiseStd.*gpsPositionNoise;
    gpsVelocityHold = xTrue(4:6) ...
        + gpsVelocityNoiseStd*gpsVelocityNoise;
end

% Barometer: 20 Hz at a 200 Hz base rate.
barometerNew = mod(sampleCounter,10.0) == 0.0;
if barometerNew
    [barometerNoise,randomState] = nextGaussian(randomState);
    barometerHold = xTrue(3) + barometerNoiseStd*barometerNoise;
end

sampleCounter = sampleCounter + 1.0;

measurement = [gyroscope; accelerometer; magnetometer; ...
               gpsPositionHold; gpsVelocityHold; barometerHold];
sensorLog = [measurement; double(gpsNew); double(barometerNew)];
end

function R = eulerToRotation(roll,pitch,yaw)
cRoll = cos(roll);
sRoll = sin(roll);
cPitch = cos(pitch);
sPitch = sin(pitch);
cYaw = cos(yaw);
sYaw = sin(yaw);
R = [ ...
    cYaw*cPitch, cYaw*sPitch*sRoll-sYaw*cRoll, cYaw*sPitch*cRoll+sYaw*sRoll; ...
    sYaw*cPitch, sYaw*sPitch*sRoll+cYaw*cRoll, sYaw*sPitch*cRoll-cYaw*sRoll; ...
    -sPitch,     cPitch*sRoll,                    cPitch*cRoll];
end

function [normalSample,state] = nextGaussian(state)
% Deterministic Box-Muller generator makes every test repeatable.
[u1,state] = nextUniform(state);
[u2,state] = nextUniform(state);
u1 = max(u1,1.0e-12);
normalSample = sqrt(-2.0*log(u1))*cos(2.0*pi*u2);
end

function [uniformSample,state] = nextUniform(state)
modulus = 4294967296.0;
state = mod(1664525.0*state + 1013904223.0,modulus);
uniformSample = (state+1.0)/(modulus+1.0);
end
