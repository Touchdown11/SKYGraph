%% PLOTSTAGE11CBF Plot CBF interventions and safety diagnostics
if ~exist('out','var')
    error('Run first: out = sim("quadrotor_stage11_cbf");');
end

[t,nominal] = readStage11Log(out,'cbf11_nominal_log',3);
[~,safe] = readStage11Log(out,'cbf11_safe_log',3);
[~,intervention] = readStage11Log(out,'cbf11_intervention_log',1);
[~,slack] = readStage11Log(out,'cbf11_slack_log',1);
[~,barrier] = readStage11Log(out,'cbf11_barrier_log',1);
[~,separation] = readStage11Log(out,'cbf11_separation_log',1);
[~,constraintCount] = readStage11Log(out,'cbf11_constraint_count_log',1);
[~,bindingCount] = readStage11Log(out,'cbf11_binding_count_log',1);
[~,solverStatus] = readStage11Log(out,'cbf11_solver_status_log',1);
[~,violation] = readStage11Log(out,'cbf11_violation_log',1);

figure('Name','Stage 11 nominal versus safe acceleration');
axisNames={'X','Y','Z'};
for axis=1:3
    subplot(3,1,axis);
    plot(t,nominal(:,axis),'--','LineWidth',1.1);
    hold on;
    plot(t,safe(:,axis),'LineWidth',1.3);
    grid on;
    ylabel(['a_',axisNames{axis},' (m/s^2)']);
    if axis==1
        legend('Nominal','CBF safe','Location','best');
        title('CBF modifies the trajectory command only when needed');
    end
end
xlabel('Time (s)');

figure('Name','Stage 11 CBF activity');
subplot(3,1,1);
stairs(t,intervention,'LineWidth',1.3);
grid on;
ylabel('Intervention');
subplot(3,1,2);
stairs(t,constraintCount,'LineWidth',1.3);
hold on;
stairs(t,bindingCount,'--','LineWidth',1.2);
grid on;
ylabel('Constraints');
legend('Available CBF rows','Binding rows','Location','best');
subplot(3,1,3);
plot(t,vecnorm(safe-nominal,2,2),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('||safe-nominal||');

figure('Name','Stage 11 safety margins');
subplot(3,1,1);
plot(t,barrier,'LineWidth',1.3);
hold on;
yline(0,'--r','Barrier boundary');
grid on;
ylabel('Minimum h');
subplot(3,1,2);
plot(t,separation,'LineWidth',1.3);
grid on;
ylabel('Minimum clearance (m)');
subplot(3,1,3);
plot(t,slack,'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('QP slack');

figure('Name','Stage 11 solver health');
subplot(2,1,1);
stairs(t,solverStatus,'LineWidth',1.3);
grid on;
ylabel('Status');
yticks([0,1]);
yticklabels({'QP','Fallback'});
subplot(2,1,2);
semilogy(t,max(violation,1e-12),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Max constraint violation');

interventionRate=100*mean(intervention>0.5);
fallbackCount=sum(solverStatus>0.5);
fprintf('\nSTAGE 11 RUN SUMMARY\n');
fprintf('Intervention rate: %.2f %%\n',interventionRate);
fprintf('Maximum slack: %.4f\n',max(slack));
fprintf('Minimum barrier: %.4f\n',min(barrier));
fprintf('Minimum clearance: %.4f m\n',min(separation));
fprintf('Fallback samples: %d\n',fallbackCount);
fprintf('Maximum reported violation: %.3e\n\n',max(violation));

function [time,data] = readStage11Log(simulationOutput,name,width)
names=simulationOutput.who;
if any(strcmp(names,name))
    signal=simulationOutput.get(name);
elseif evalin('base',['exist(''' name ''',''var'')'])
    signal=evalin('base',name);
else
    error('Missing log "%s".',name);
end
time=signal.Time(:);
data=squeeze(signal.Data);
if isvector(data)
    data=data(:);
elseif size(data,1)==numel(time)
    % Correct orientation.
elseif size(data,2)==numel(time)
    data=data.';
else
    error('%s has incompatible dimensions.',name);
end
if size(data,2)<width
    error('%s needs %d columns but has %d.',name,width,size(data,2));
end
data=data(:,1:width);
end
