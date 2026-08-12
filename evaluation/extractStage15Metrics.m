function row=extractStage15Metrics(out,configuration)
%EXTRACTSTAGE15METRICS Convert one simulation output into one table row.

[tTrue,xTrue]=readAny15(out,{'true_state15_log','true_state8_log','true_state4w_log'},16);
[tTrajectory,trajectory]=readAny15(out,{'trajectory15_log','trajectory4w_log'},10);
[tApplied,applied]=readAny15(out,{'stage15_applied_log','cbf11_safe_log'},3);
[~,separation]=readAny15(out,{'cbf11_separation_log'},1);
[~,intervention]=readAny15(out,{'cbf11_intervention_log'},1);
[~,slack]=readAny15(out,{'cbf11_slack_log'},1);
[~,solverStatus]=readAny15(out,{'cbf11_solver_status_log'},1);
[~,packetAge]=readAny15(out,{'tof8_packet_age_log'},1);
[~,graphMask]=readAny15(out,{'graph10_mask_log'},17);
[~,edgeCount]=readAny15(out,{'graph10_edge_count_log'},1);
[~,gatTopScore]=readAny15(out,{'gat12_top_score_log'},5);
[tScenario,scenarioPosition]=readAny15(out,{'scenario8_position_log'},48);
[~,scenarioRadius]=readAny15(out,{'scenario8_radius_log'},16);
[~,scenarioActive]=readAny15(out,{'scenario8_active_log'},16);

trajectoryAtTrue=interp15(tTrajectory,trajectory,tTrue,'linear');
scenarioPosition=interp15(tScenario,scenarioPosition,tTrue,'previous');
scenarioRadius=interp15(tScenario,scenarioRadius,tTrue,'previous');
scenarioActive=interp15(tScenario,scenarioActive,tTrue,'previous')>0.5;
finalGoalError=norm(xTrue(end,1:3)-trajectoryAtTrue(end,1:3));
positionDifference=diff(xTrue(:,1:3),1,1);
pathLength=sum(vecnorm(positionDifference,2,2));
if numel(tApplied)>1
    controlEffort=trapz(tApplied,sum(applied.^2,2));
else
    controlEffort=0;
end
% True collision clearance is evaluated from scenario ground truth. The
% CBF tracked-separation log can include the ground/ceiling and uncertain
% ToF surface tracks, so it must not be used as physical collision truth.
egoRadius=0.22;
trueClearance=100*ones(numel(tTrue),1);
for sample=1:numel(tTrue)
    positions=reshape(scenarioPosition(sample,:).',3,16);
    for entity=1:16
        if scenarioActive(sample,entity)
            clearance=norm(positions(:,entity)-xTrue(sample,1:3).') ...
                -(egoRadius+scenarioRadius(sample,entity));
            trueClearance(sample)=min(trueClearance(sample),clearance);
        end
    end
end
minimumSeparation=min(trueClearance);
trackedMinimumSeparation=min(separation);
violationFraction=mean(trueClearance<0);
missionSuccess=finalGoalError<0.30 && minimumSeparation>=0;

row=table(string(configuration.Name),configuration.PolicyMode, ...
    configuration.SafetyMode,configuration.ScenarioMode, ...
    configuration.FaultSchedule,missionSuccess,minimumSeparation, ...
    trackedMinimumSeparation,violationFraction,finalGoalError,pathLength,controlEffort, ...
    mean(intervention>0.5),max(slack),sum(solverStatus>0.5), ...
    mean(packetAge>0.15),mean(sum(graphMask>0.5,2)),mean(edgeCount), ...
    mean(gatTopScore(:,1)),max(xTrue(:,13:16),[],'all'), ...
    'VariableNames',{'Name','PolicyMode','SafetyMode','ScenarioMode', ...
    'FaultSchedule','MissionSuccess','MinimumSeparation', ...
    'TrackedMinimumSeparation','ViolationFraction','FinalGoalError','PathLength','ControlEffort', ...
    'CBFInterventionRate','MaximumSlack','FallbackCount', ...
    'StalePacketFraction','MeanGraphNodes','MeanGraphEdges', ...
    'MeanTopGATScore','MaximumMotorSpeed'});
end

function [time,data]=readAny15(out,candidates,width)
names=out.who;signal=[];resolved='';
for k=1:numel(candidates)
    name=char(candidates{k});
    if any(strcmp(names,name)),signal=out.get(name);resolved=name;break;end
end
if isempty(signal),error('Missing required metric log: %s',strjoin(string(candidates),', '));end
time=signal.Time(:);data=squeeze(signal.Data);
if ~(isa(data,'double')||isa(data,'single')),data=double(data);end
if isvector(data),data=data(:);
elseif size(data,1)==numel(time)
elseif size(data,2)==numel(time),data=data.';
else,error('%s has incompatible dimensions.',resolved);end
if size(data,2)<width,error('%s needs %d columns.',resolved,width);end
data=data(:,1:width);
end

function result=interp15(time,data,newTime,method)
[uniqueTime,index]=unique(time,'stable');result=interp1(uniqueTime,data(index,:),newTime,method,'extrap');
end
