%% TESTCBFSTAGE11 Validate controller split and CBF-QP behavior
clear;
clc;
close all;

relativePosition=zeros(3,16);
relativeVelocity=zeros(3,16);
radius=zeros(16,1);
active=false(16,1);
confidence=zeros(16,1);
age=zeros(16,1);
egoVelocity=zeros(3,1);
nominal=[1.0;0.0;0.0];

% No threat: command should pass through.
[safe,intervention,slack,~,~,count,binding,status,violation] = ...
    cbfSafetyShieldStage11(nominal,egoVelocity,relativePosition, ...
    relativeVelocity,radius,active,confidence,age);
assert(norm(safe-nominal)<1e-8);
assert(~intervention && slack<1e-8 && count==0 && binding==0);
assert(status==0 && violation<1e-6);

% Close approaching unknown directly ahead.
relativePosition(:,1)=[1.0;0;0];
relativeVelocity(:,1)=[-0.5;0;0];
radius(1)=0.15; active(1)=true; confidence(1)=0.95;
[safeAhead,interventionAhead,slackAhead,hAhead,separationAhead,countAhead, ...
    bindingAhead,statusAhead,violationAhead] = ...
    cbfSafetyShieldStage11(nominal,egoVelocity,relativePosition, ...
    relativeVelocity,radius,active,confidence,age);
assert(interventionAhead);
assert(safeAhead(1)<nominal(1));
assert(countAhead==1 && bindingAhead>=1);
assert(statusAhead==0 && violationAhead<1e-3);
assert(isfinite(slackAhead) && isfinite(hAhead) && separationAhead>0);

% Side threat should influence lateral acceleration.
relativePosition(:,1)=[0.0;0.75;0.0];
relativeVelocity(:,1)=[0.0;-0.4;0.0];
nominalSide=[0.0;1.0;0.0];
[safeSide,interventionSide] = cbfSafetyShieldStage11(nominalSide, ...
    egoVelocity,relativePosition,relativeVelocity,radius,active,confidence,age);
assert(interventionSide && safeSide(2)<nominalSide(2));

% Dense 16-constraint stress test remains finite and bounded.
for k=1:16
    angle=2*pi*(k-1)/16;
    relativePosition(:,k)=[1.0+0.03*k;0;0];
    relativePosition(1,k)=(1.0+0.03*k)*cos(angle);
    relativePosition(2,k)=(1.0+0.03*k)*sin(angle);
    relativeVelocity(:,k)=-0.15*relativePosition(:,k)/norm(relativePosition(:,k));
    radius(k)=0.10;
    active(k)=true;
    confidence(k)=0.8;
    age(k)=0.05;
end
[safeDense,~,slackDense,~,~,countDense,~,statusDense,violationDense] = ...
    cbfSafetyShieldStage11([2;-2;1],egoVelocity,relativePosition, ...
    relativeVelocity,radius,active,confidence,age);
assert(all(isfinite(safeDense)) && isfinite(slackDense));
assert(all(safeDense<=[3;3;5]) && all(safeDense>=[-3;-3;-5]));
assert(countDense==16);
assert(statusDense==0 && violationDense<1e-3);

% Verify outer and inner controller split produces finite motor commands.
xEstimate=zeros(12,1);
trajectory=zeros(10,1); trajectory(3)=1.0;
positionIntegral=zeros(3,1);
[nominalAcceleration,yawDesired]=nominalAccelerationStage11( ...
    xEstimate,trajectory,positionIntegral,true);
omega=attitudeMotorControllerStage11(xEstimate,nominalAcceleration, ...
    yawDesired,true);
assert(isequal(size(omega),[4,1]));
assert(all(isfinite(omega)) && all(omega>=0) && all(omega<=900));

fprintf('\nSTAGE 11 CBF-QP VALIDATION\n');
fprintf('==========================\n');
fprintf('No-threat pass-through: PASS\n');
fprintf('Ahead-threat intervention: PASS (ax %.3f -> %.3f)\n',1.0,safeAhead(1));
fprintf('Side-threat intervention: PASS (ay %.3f -> %.3f)\n',1.0,safeSide(2));
fprintf('Dense constraints: PASS (%d CBF rows, slack %.4f)\n', ...
    countDense,slackDense);
fprintf('Controller split and motor mixer: PASS\n');
fprintf('RESULT: PASS - Stage 11 CBF safety shield is ready.\n\n');

figure('Name','Stage 11 CBF command comparison');
bar([nominal,safeAhead]);
grid on;
xticklabels({'a_x','a_y','a_z'});
ylabel('Acceleration command (m/s^2)');
legend('Nominal','CBF safe','Location','best');
title('Approaching obstacle directly ahead');
