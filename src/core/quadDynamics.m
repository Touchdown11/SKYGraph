function dx = quadDynamics(x, omega)
%QUADDYNAMICS Nonlinear 12-state quadrotor dynamics built from equations.
% Coordinate frame: world X forward, Y left, Z up.
%
% State vector x:
%   1:3   world position [x; y; z] (m)
%   4:6   world velocity [vx; vy; vz] (m/s)
%   7:9   Euler angles [roll; pitch; yaw] (rad)
%   10:12 body angular rates [p; q; r] (rad/s)
%
% Motor order:
%   1 front (+X), 2 left (+Y), 3 rear (-X), 4 right (-Y)
% omega contains motor speeds in rad/s.

%#codegen

% Physical parameters (keep these consistent with flightController.m)
m = 1.20;          % mass (kg)
g = 9.81;          % gravitational acceleration (m/s^2)
arm = 0.23;        % center-to-motor distance (m)
kf = 1.8e-5;       % thrust coefficient (N/(rad/s)^2)
km = 2.5e-7;       % yaw moment coefficient (N*m/(rad/s)^2)
Ixx = 0.020;        % roll inertia (kg*m^2)
Iyy = 0.020;        % pitch inertia (kg*m^2)
Izz = 0.040;        % yaw inertia (kg*m^2)
cTrans = 0.15;     % simple translational drag (N/(m/s))
cRot = 0.020;      % simple rotational drag (N*m/(rad/s))
omegaMax = 900.0;  % motor speed limit (rad/s)

% Protect the model from impossible motor commands.
omega = min(max(omega, 0.0), omegaMax);
s = omega .* omega;

% Total thrust and body torques.
totalThrust = kf * (s(1) + s(2) + s(3) + s(4));
tauX = arm * kf * (s(2) - s(4));
tauY = arm * kf * (s(3) - s(1));
tauZ = km * (s(1) - s(2) + s(3) - s(4));

velocity = x(4:6);
roll  = x(7);
pitch = x(8);
yaw   = x(9);
pBody = x(10);
qBody = x(11);
rBody = x(12);

% Rotation matrix from body coordinates to world coordinates: Rz*Ry*Rx.
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

% Translational dynamics in the world frame.
thrustWorld = RbodyToWorld * [0.0; 0.0; totalThrust];
acceleration = thrustWorld / m + [0.0; 0.0; -g] ...
    - (cTrans / m) * velocity;

% Convert body angular rates [p q r] to Euler-angle rates.
% Euler angles have a mathematical singularity at pitch = +/-90 degrees.
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

% Rotational rigid-body equations: I*wDot + w x (I*w) = torque.
bodyRateDot = zeros(3,1);
bodyRateDot(1) = (tauX - (Izz - Iyy)*qBody*rBody - cRot*pBody) / Ixx;
bodyRateDot(2) = (tauY - (Ixx - Izz)*pBody*rBody - cRot*qBody) / Iyy;
bodyRateDot(3) = (tauZ - (Iyy - Ixx)*pBody*qBody - cRot*rBody) / Izz;

% Assemble the state derivative.
dx = zeros(12,1);
dx(1:3) = velocity;
dx(4:6) = acceleration;
dx(7:9) = eulerRate;
dx(10:12) = bodyRateDot;
end
