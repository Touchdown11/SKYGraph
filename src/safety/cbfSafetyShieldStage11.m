function [safeAcceleration,intervention,slack,minBarrier,minSeparation, ...
    cbfConstraintCount,bindingCount,solverStatus,maxViolation] = ...
    cbfSafetyShieldStage11(nominalAcceleration,egoVelocity, ...
    relativePosition,relativeVelocity,trackRadius,trackActive, ...
    confidence,age)
%CBFSAFETYSHIELDSTAGE11 Fixed-size CBF quadratic-program projection.
%
% Uses a relative-degree-two exponential CBF and a fixed-iteration
% Hildreth dual QP solver suitable for a MATLAB Function block.
% solverStatus: 0 normal, 1 fallback braking used.

%#codegen

maximumConstraints=24;
variableCount=4; % [ax ay az slack]
A=zeros(maximumConstraints,variableCount);
b=zeros(maximumConstraints,1);
constraintCount=0;
cbfConstraintCount=0;
minBarrier=100.0;
minSeparation=100.0;

egoRadius=0.22;
baseMargin=0.18;
k1=2.0;
k2=2.0;

% CBF constraints for all active tracks within the graph/safety horizon.
for track=1:16
    if ~trackActive(track) || confidence(track)<0.05
        continue;
    end
    r=zeros(3,1); w=zeros(3,1);
    r(1)=relativePosition(1,track); r(2)=relativePosition(2,track); r(3)=relativePosition(3,track);
    w(1)=relativeVelocity(1,track); w(2)=relativeVelocity(2,track); w(3)=relativeVelocity(3,track);
    distance=sqrt(r(1)^2+r(2)^2+r(3)^2);
    if distance>3.5 || distance<1.0e-5
        continue;
    end
    uncertainty=0.20*(1.0-min(max(confidence(track),0.0),1.0)) ...
        +0.15*min(max(age(track)/0.50,0.0),1.0);
    safeDistance=egoRadius+trackRadius(track)+baseMargin+uncertainty;
    h=distance^2-safeDistance^2;
    hDot=2.0*(r(1)*w(1)+r(2)*w(2)+r(3)*w(3));
    relativeSpeedSquared=w(1)^2+w(2)^2+w(3)^2;
    rhs=2.0*relativeSpeedSquared+(k1+k2)*hDot+k1*k2*h;

    constraintCount=constraintCount+1;
    cbfConstraintCount=cbfConstraintCount+1;
    A(constraintCount,1)=2.0*r(1);
    A(constraintCount,2)=2.0*r(2);
    A(constraintCount,3)=2.0*r(3);
    A(constraintCount,4)=-1.0; % nonnegative slack relaxes CBF
    b(constraintCount)=rhs;
    minBarrier=min(minBarrier,h);
    minSeparation=min(minSeparation,distance-trackRadius(track)-egoRadius);
end

% Acceleration box constraints.
uMaximum=[3.0;3.0;5.0];
uMinimum=[-3.0;-3.0;-5.0];
for axis=1:3
    constraintCount=constraintCount+1;
    A(constraintCount,axis)=1.0;
    b(constraintCount)=uMaximum(axis);
    constraintCount=constraintCount+1;
    A(constraintCount,axis)=-1.0;
    b(constraintCount)=-uMinimum(axis);
end
% Slack bounds 0 <= slack <= 20.
constraintCount=constraintCount+1;
A(constraintCount,4)=-1.0;
b(constraintCount)=0.0;
constraintCount=constraintCount+1;
A(constraintCount,4)=1.0;
b(constraintCount)=20.0;

% QP: min 0.5*(z-zNom)'H*(z-zNom), A*z<=b.
zNominal=zeros(variableCount,1);
zNominal(1:3)=min(max(nominalAcceleration,uMinimum),uMaximum);
HInverse=[1.0;1.0;1.0;1.0/200.0]; % high slack penalty
P=zeros(maximumConstraints,maximumConstraints);
q=zeros(maximumConstraints,1);
for i=1:constraintCount
    q(i)=A(i,:)*zNominal-b(i);
    for j=1:constraintCount
        value=0.0;
        for variable=1:variableCount
            value=value+A(i,variable)*HInverse(variable)*A(j,variable);
        end
        P(i,j)=value;
    end
end

lambda=zeros(maximumConstraints,1);
for iteration=1:300
    for i=1:constraintCount
        diagonal=P(i,i);
        if diagonal>1.0e-12
            sumOther=0.0;
            for j=1:constraintCount
                if j~=i
                    sumOther=sumOther+P(i,j)*lambda(j);
                end
            end
            lambda(i)=max(0.0,(q(i)-sumOther)/diagonal);
        end
    end
end

correction=zeros(variableCount,1);
for variable=1:variableCount
    for i=1:constraintCount
        correction(variable)=correction(variable) ...
            +HInverse(variable)*A(i,variable)*lambda(i);
    end
end
z=zNominal-correction;
z(1:3)=min(max(z(1:3),uMinimum),uMaximum);
z(4)=min(max(z(4),0.0),20.0);

maxViolation=0.0;
for i=1:constraintCount
    violation=A(i,:)*z-b(i);
    maxViolation=max(maxViolation,violation);
end

solverStatus=0.0;
if any(~isfinite(z)) || maxViolation>1.0e-3
    % Deterministic fallback: oppose estimated ego velocity.
    safeAcceleration=zeros(3,1);
    safeAcceleration(1)=min(max(-1.5*egoVelocity(1),uMinimum(1)),uMaximum(1));
    safeAcceleration(2)=min(max(-1.5*egoVelocity(2),uMinimum(2)),uMaximum(2));
    safeAcceleration(3)=min(max(-1.2*egoVelocity(3),uMinimum(3)),uMaximum(3));
    slack=20.0;
    solverStatus=1.0;
else
    safeAcceleration=z(1:3);
    slack=z(4);
end

intervention=norm(safeAcceleration-zNominal(1:3))>0.02;
bindingCount=0.0;
for i=1:cbfConstraintCount
    if lambda(i)>1.0e-5
        bindingCount=bindingCount+1.0;
    end
end
end
