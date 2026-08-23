function [xEstimate, estimatorLog] = stateEstimatorStage3( ...
    measurement, gpsNew, barometerNew)
%STATEESTIMATORSTAGE3 Custom attitude and position/velocity estimator.
%
% Runs at 200 Hz (Ts = 0.005 s). Attitude uses quaternion gyro
% propagation plus motion-gated accelerometer/magnetometer correction.
% Each position axis uses a two-state [position; velocity] Kalman filter.

%#codegen

persistent quaternion positionEstimate velocityEstimate ...
    accelerationWorldFiltered gyroBiasEstimate sampleCounter ...
    covariance initialized

Ts = 0.005;
g = 9.81;

if isempty(initialized)
    quaternion = [1.0;0.0;0.0;0.0];
    positionEstimate = zeros(3,1);
    velocityEstimate = zeros(3,1);
    accelerationWorldFiltered = zeros(3,1);
    gyroBiasEstimate = zeros(3,1);
    sampleCounter = 0.0;
    covariance = zeros(2,2,3);
    for axis = 1:3
        covariance(:,:,axis) = [1.0,0.0;0.0,1.0];
    end
    initialized = true;
end

gyroscope = measurement(1:3);
accelerometer = measurement(4:6);
magnetometer = measurement(7:9);
gpsPosition = measurement(10:12);
gpsVelocity = measurement(13:15);
barometerAltitude = measurement(16);

% Build an absolute attitude observation using gravity and magnetic north.
[measurementQuaternion,attitudeMeasurementValid] = ...
    triadQuaternion(accelerometer,magnetometer);

% Initialize the estimator from the first available measurements.
if sampleCounter == 0.0
    if attitudeMeasurementValid
        quaternion = measurementQuaternion;
    end
    if gpsNew
        positionEstimate = gpsPosition;
        velocityEstimate = gpsVelocity;
    end
    if barometerNew
        positionEstimate(3) = barometerAltitude;
    end
end

% First second: the flight controller holds equal hover motor speeds while
% this running average estimates the stationary gyroscope bias.
if sampleCounter < 200.0
    gyroBiasEstimate = (sampleCounter*gyroBiasEstimate + gyroscope) ...
        /(sampleCounter+1.0);
end
sampleCounter = sampleCounter + 1.0;

gyroscopeCorrected = gyroscope - gyroBiasEstimate;

% Quaternion propagation from body angular rate.
omegaQuaternion = [0.0;gyroscopeCorrected];
quaternionDerivative = 0.5*quaternionMultiply(quaternion,omegaQuaternion);
quaternionPredicted = normalizeQuaternion( ...
    quaternion + Ts*quaternionDerivative);

% Accelerometers are not pure tilt sensors during translation. Apply a
% stronger correction only when predicted world acceleration is small.
Rpredicted = quaternionToRotation(quaternionPredicted);
predictedMotionAcceleration = Rpredicted*accelerometer + [0.0;0.0;-g];

accelerometerMagnitude = norm(accelerometer);
if attitudeMeasurementValid ...
        && accelerometerMagnitude > 7.5 ...
        && accelerometerMagnitude < 12.5
    if dot(quaternionPredicted,measurementQuaternion) < 0.0
        measurementQuaternion = -measurementQuaternion;
    end
    if norm(predictedMotionAcceleration) < 0.5
        correctionWeight = 0.008;
    else
        correctionWeight = 0.0005;
    end
    quaternion = normalizeQuaternion( ...
        (1.0-correctionWeight)*quaternionPredicted ...
        + correctionWeight*measurementQuaternion);
else
    quaternion = quaternionPredicted;
end

RbodyToWorld = quaternionToRotation(quaternion);
accelerationWorld = RbodyToWorld*accelerometer + [0.0;0.0;-g];
accelerationWorldFiltered = 0.85*accelerationWorldFiltered ...
                            + 0.15*accelerationWorld;

% Three independent linear Kalman filters, each with state [position;velocity].
F = [1.0,Ts;0.0,1.0];
B = [0.5*Ts*Ts;Ts];
identity2 = eye(2);
accelerationProcessStd = [0.50;0.50;0.70];
gpsPositionStd = [0.15;0.15;0.25];
gpsVelocityStd = 0.05;
barometerStd = 0.05;

for axis = 1:3
    axisState = [positionEstimate(axis);velocityEstimate(axis)];
    axisState = F*axisState + B*accelerationWorldFiltered(axis);

    qScale = accelerationProcessStd(axis)^2;
    processNoise = qScale*[Ts^4/4.0,Ts^3/2.0; ...
                           Ts^3/2.0,Ts^2] ...
                   + [1.0e-7,0.0;0.0,1.0e-5];
    axisCovariance = F*covariance(:,:,axis)*F.' + processNoise;

    if gpsNew
        gpsNoise = [gpsPositionStd(axis)^2,0.0; ...
                    0.0,gpsVelocityStd^2];
        innovation = [gpsPosition(axis);gpsVelocity(axis)]-axisState;
        innovationCovariance = axisCovariance + gpsNoise;
        kalmanGain = axisCovariance/innovationCovariance;
        axisState = axisState + kalmanGain*innovation;
        axisCovariance = (identity2-kalmanGain)*axisCovariance;
    end

    if barometerNew && axis == 3
        H = [1.0,0.0];
        barometerInnovation = barometerAltitude-H*axisState;
        barometerInnovationCovariance = ...
            H*axisCovariance*H.' + barometerStd^2;
        barometerGain = axisCovariance*H.'/barometerInnovationCovariance;
        axisState = axisState + barometerGain*barometerInnovation;
        axisCovariance = (identity2-barometerGain*H)*axisCovariance;
    end

    % Maintain symmetry against small floating-point roundoff.
    axisCovariance = 0.5*(axisCovariance+axisCovariance.');
    positionEstimate(axis) = axisState(1);
    velocityEstimate(axis) = axisState(2);
    covariance(:,:,axis) = axisCovariance;
end

attitudeEstimate = rotationToEuler(RbodyToWorld);

xEstimate = zeros(12,1);
xEstimate(1:3) = positionEstimate;
xEstimate(4:6) = velocityEstimate;
xEstimate(7:9) = attitudeEstimate;
xEstimate(10:12) = gyroscopeCorrected;

estimatorLog = [gyroBiasEstimate; ...
    covariance(1,1,1);covariance(1,1,2);covariance(1,1,3); ...
    accelerationWorldFiltered];
end

function [q,valid] = triadQuaternion(accelerometer,magnetometer)
q = [1.0;0.0;0.0;0.0];
valid = false;
accelerationNorm = norm(accelerometer);
if accelerationNorm < 1.0e-6
    return;
end

% At rest, accelerometer direction is world +Z expressed in body axes.
upBody = accelerometer/accelerationNorm;
northBody = magnetometer-dot(magnetometer,upBody)*upBody;
if norm(northBody) < 0.05
    return;
end
northBody = northBody/norm(northBody);
leftBody = cross(upBody,northBody);
if norm(leftBody) < 1.0e-6
    return;
end
leftBody = leftBody/norm(leftBody);
northBody = cross(leftBody,upBody);
northBody = northBody/norm(northBody);

% Columns are world basis vectors expressed in body axes: this is R'.
RworldToBody = [northBody,leftBody,upBody];
RbodyToWorld = RworldToBody.';
q = rotationToQuaternion(RbodyToWorld);
valid = true;
end

function product = quaternionMultiply(a,b)
product = zeros(4,1);
product(1) = a(1)*b(1)-dot(a(2:4),b(2:4));
product(2:4) = a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4));
end

function q = normalizeQuaternion(q)
qNorm = norm(q);
if qNorm < 1.0e-12
    q = [1.0;0.0;0.0;0.0];
else
    q = q/qNorm;
end
end

function R = quaternionToRotation(q)
q = normalizeQuaternion(q);
w = q(1);
x = q(2);
y = q(3);
z = q(4);
R = [1.0-2.0*(y*y+z*z), 2.0*(x*y-z*w),     2.0*(x*z+y*w); ...
     2.0*(x*y+z*w),     1.0-2.0*(x*x+z*z), 2.0*(y*z-x*w); ...
     2.0*(x*z-y*w),     2.0*(y*z+x*w),     1.0-2.0*(x*x+y*y)];
end

function q = rotationToQuaternion(R)
traceR = R(1,1)+R(2,2)+R(3,3);
q = zeros(4,1);
if traceR > 0.0
    S = 2.0*sqrt(traceR+1.0);
    q(1) = 0.25*S;
    q(2) = (R(3,2)-R(2,3))/S;
    q(3) = (R(1,3)-R(3,1))/S;
    q(4) = (R(2,1)-R(1,2))/S;
elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
    S = 2.0*sqrt(1.0+R(1,1)-R(2,2)-R(3,3));
    q(1) = (R(3,2)-R(2,3))/S;
    q(2) = 0.25*S;
    q(3) = (R(1,2)+R(2,1))/S;
    q(4) = (R(1,3)+R(3,1))/S;
elseif R(2,2) > R(3,3)
    S = 2.0*sqrt(1.0+R(2,2)-R(1,1)-R(3,3));
    q(1) = (R(1,3)-R(3,1))/S;
    q(2) = (R(1,2)+R(2,1))/S;
    q(3) = 0.25*S;
    q(4) = (R(2,3)+R(3,2))/S;
else
    S = 2.0*sqrt(1.0+R(3,3)-R(1,1)-R(2,2));
    q(1) = (R(2,1)-R(1,2))/S;
    q(2) = (R(1,3)+R(3,1))/S;
    q(3) = (R(2,3)+R(3,2))/S;
    q(4) = 0.25*S;
end
q = normalizeQuaternion(q);
end

function euler = rotationToEuler(R)
euler = zeros(3,1);
euler(1) = atan2(R(3,2),R(3,3));
sinePitch = min(max(-R(3,1),-1.0),1.0);
euler(2) = asin(sinePitch);
euler(3) = atan2(R(2,1),R(1,1));
end
