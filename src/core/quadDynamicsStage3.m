function dx = quadDynamicsStage3(x, omegaCommand, windWorld, ...
    externalForceWorld, externalTorqueBody)
%QUADDYNAMICSSTAGE3 16-state quadrotor with motor lag and wind drag.
%
% x(1:12)  = position, velocity, Euler attitude, and body angular rates
% x(13:16) = actual motor speeds [front; left; rear; right] (rad/s)
%
% windWorld is the velocity of the air relative to the ground (m/s).
% externalForceWorld is an additional disturbance force in world axes (N).
% externalTorqueBody is an additional disturbance torque in body axes (N*m).

%#codegen

% Physical parameters (keep common values consistent with the controller)
m = 1.20;
g = 9.81;
arm = 0.23;
kf = 1.8e-5;
km = 2.5e-7;
Ixx = 0.020;
Iyy = 0.020;
Izz = 0.040;
cRot = 0.020;
omegaMax = 900.0;
motorTimeConstant = 0.060;  % seconds

% Direction-dependent teaching-model drag coefficients in body axes.
linearDrag = [0.25; 0.25; 0.35];
quadraticDrag = [0.12; 0.12; 0.18];

% Actual motor speeds are states; commanded speeds are controller outputs.
omegaActual = min(max(x(13:16), 0.0), omegaMax);
omegaCommand = min(max(omegaCommand, 0.0), omegaMax);
s = omegaActual .* omegaActual;

% Rotor thrust and body torque.
totalThrust = kf * (s(1) + s(2) + s(3) + s(4));
tauRotor = zeros(3,1);
tauRotor(1) = arm*kf*(s(2) - s(4));
tauRotor(2) = arm*kf*(s(3) - s(1));
tauRotor(3) = km*(s(1) - s(2) + s(3) - s(4));
totalTorqueBody = tauRotor + externalTorqueBody;

velocity = x(4:6);
roll  = x(7);
pitch = x(8);
yaw   = x(9);
pBody = x(10);
qBody = x(11);
rBody = x(12);

% Rotation matrix from body axes to world axes: Rz*Ry*Rx.
cRoll = cos(roll);
sRoll = sin(roll);
cPitch = cos(pitch);
sPitch = sin(pitch);
cYaw = cos(yaw);
sYaw = sin(yaw);

RbodyToWorld = [ ...
    cYaw*cPitch, cYaw*sPitch*sRoll - sYaw*cRoll, cYaw*sPitch*cRoll + sYaw*sRoll; ...
    sYaw*cPitch, sYaw*sPitch*sRoll + cYaw*cRoll, sYaw*sPitch*cRoll - cYaw*sRoll; ...
    -sPitch,     cPitch*sRoll,                         cPitch*cRoll];

% Aerodynamic drag depends on drone velocity relative to moving air.
relativeAirVelocityWorld = velocity - windWorld;
relativeAirVelocityBody = RbodyToWorld.' * relativeAirVelocityWorld;
dragForceBody = -linearDrag .* relativeAirVelocityBody ...
                -quadraticDrag .* abs(relativeAirVelocityBody) ...
                 .* relativeAirVelocityBody;
dragForceWorld = RbodyToWorld * dragForceBody;

thrustWorld = RbodyToWorld * [0.0; 0.0; totalThrust];
acceleration = (thrustWorld + dragForceWorld + externalForceWorld) / m ...
               + [0.0; 0.0; -g];

% Euler angle kinematics, guarded near the pitch singularity.
safeCosPitch = cPitch;
if abs(safeCosPitch) < 0.05
    if safeCosPitch >= 0.0
        safeCosPitch = 0.05;
    else
        safeCosPitch = -0.05;
    end
end
tanPitch = sPitch / safeCosPitch;

eulerRate = zeros(3,1);
eulerRate(1) = pBody + qBody*sRoll*tanPitch + rBody*cRoll*tanPitch;
eulerRate(2) = qBody*cRoll - rBody*sRoll;
eulerRate(3) = qBody*sRoll/safeCosPitch + rBody*cRoll/safeCosPitch;

% Rigid-body rotational dynamics.
bodyRateDot = zeros(3,1);
bodyRateDot(1) = (totalTorqueBody(1) ...
    - (Izz-Iyy)*qBody*rBody - cRot*pBody) / Ixx;
bodyRateDot(2) = (totalTorqueBody(2) ...
    - (Ixx-Izz)*pBody*rBody - cRot*qBody) / Iyy;
bodyRateDot(3) = (totalTorqueBody(3) ...
    - (Iyy-Ixx)*pBody*qBody - cRot*rBody) / Izz;

% First-order motor dynamics: 63% response after one time constant.
motorSpeedDot = (omegaCommand - omegaActual) / motorTimeConstant;
for motor = 1:4
    if x(12+motor) <= 0.0 && motorSpeedDot(motor) < 0.0
        motorSpeedDot(motor) = 0.0;
    elseif x(12+motor) >= omegaMax && motorSpeedDot(motor) > 0.0
        motorSpeedDot(motor) = 0.0;
    end
end

% Assemble the 16 plant-state derivatives.
dx = zeros(16,1);
dx(1:3) = velocity;
dx(4:6) = acceleration;
dx(7:9) = eulerRate;
dx(10:12) = bodyRateDot;
dx(13:16) = motorSpeedDot;
end
