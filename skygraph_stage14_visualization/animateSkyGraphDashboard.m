function animateSkyGraphDashboard(out,varargin)
%ANIMATESKYGRAPHDASHBOARD Multi-panel video-style SkyGraph playback.
%
% animateSkyGraphDashboard(out,'PlaybackSpeed',2,'FrameRate',20)

parser=inputParser;
addParameter(parser,'PlaybackSpeed',2,@(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(parser,'FrameRate',20,@(x)isnumeric(x)&&isscalar(x)&&x>=5);
parse(parser,varargin{:});options=parser.Results;

% Validate before allocating graphics.
validateSkyGraphStage14(out);

[tTrue,xTrue]=readDash(out,{'true_state14_log','true_state8_log','true_state4w_log'},16);
[tEst,xEst]=readDash(out,{'estimated_state14_log','estimated_state4w_log','estimated_state3_log'},12);
[tTraj,trajectory]=readDash(out,{'trajectory14_log','trajectory4w_log'},10);
[tScene,scenePosition]=readDash(out,{'scenario8_position_log'},48);
[~,sceneRadius]=readDash(out,{'scenario8_radius_log'},16);
[~,sceneType]=readDash(out,{'scenario8_type_log'},16);
[~,sceneActive]=readDash(out,{'scenario8_active_log'},16);
[tToF,tofDistance]=readDash(out,{'tof8_distance_log'},10);
[~,tofValid]=readDash(out,{'tof8_valid_log'},10);
[~,packetAge]=readDash(out,{'tof8_packet_age_log'},1);
[tTrack,trackPosition]=readDash(out,{'track9_position_log'},48);
[~,trackId]=readDash(out,{'track9_id_log'},16);
[~,trackType]=readDash(out,{'track9_type_log'},16);
[~,trackActive]=readDash(out,{'track9_active_log'},16);
[~,trackConfidence]=readDash(out,{'track9_confidence_log'},16);
[tGraph,graphMask]=readDash(out,{'graph10_mask_log'},17);
[~,graphId]=readDash(out,{'graph10_id_log'},17);
[~,graphAdjacency]=readDash(out,{'graph10_adjacency_log'},289);
[tGAT,gatThreat]=readDash(out,{'gat12_threat_log'},17);
[~,gatAttention]=readDash(out,{'gat12_attention_log'},289);
[~,gatTopId]=readDash(out,{'gat12_top_id_log'},5);
[~,gatTopScore]=readDash(out,{'gat12_top_score_log'},5);
[tPolicy,nominalAction]=readDash(out,{'ppo13_selected_log'},3);
[~,policySource]=readDash(out,{'ppo13_source_log'},1);
[tCBF,safeAction]=readDash(out,{'cbf11_safe_log'},3);
[~,intervention]=readDash(out,{'cbf11_intervention_log'},1);
[~,slack]=readDash(out,{'cbf11_slack_log'},1);
[~,separation]=readDash(out,{'cbf11_separation_log'},1);

% Align all signals to the 200 Hz true-state time base.
xEst=alignDash(tEst,xEst,tTrue,'linear');trajectory=alignDash(tTraj,trajectory,tTrue,'linear');
scenePosition=alignDash(tScene,scenePosition,tTrue,'previous');sceneRadius=alignDash(tScene,sceneRadius,tTrue,'previous');sceneType=alignDash(tScene,sceneType,tTrue,'previous');sceneActive=alignDash(tScene,sceneActive,tTrue,'previous');
tofDistance=alignDash(tToF,tofDistance,tTrue,'previous');tofValid=alignDash(tToF,tofValid,tTrue,'previous');packetAge=alignDash(tToF,packetAge,tTrue,'previous');
trackPosition=alignDash(tTrack,trackPosition,tTrue,'previous');trackId=alignDash(tTrack,trackId,tTrue,'previous');trackType=alignDash(tTrack,trackType,tTrue,'previous');trackActive=alignDash(tTrack,trackActive,tTrue,'previous');trackConfidence=alignDash(tTrack,trackConfidence,tTrue,'previous');
graphMask=alignDash(tGraph,graphMask,tTrue,'previous');graphId=alignDash(tGraph,graphId,tTrue,'previous');graphAdjacency=alignDash(tGraph,graphAdjacency,tTrue,'previous');
gatThreat=alignDash(tGAT,gatThreat,tTrue,'previous');gatAttention=alignDash(tGAT,gatAttention,tTrue,'previous');gatTopId=alignDash(tGAT,gatTopId,tTrue,'previous');gatTopScore=alignDash(tGAT,gatTopScore,tTrue,'previous');
nominalAction=alignDash(tPolicy,nominalAction,tTrue,'previous');policySource=alignDash(tPolicy,policySource,tTrue,'previous');
safeAction=alignDash(tCBF,safeAction,tTrue,'previous');intervention=alignDash(tCBF,intervention,tTrue,'previous');slack=alignDash(tCBF,slack,tTrue,'previous');separation=alignDash(tCBF,separation,tTrue,'previous');

% Figure layout: large 3-D panel, ToF panel, graph panel, timeline, status.
fig=figure('Name','SkyGraph Mission Dashboard','Color',[0.035,0.05,0.08], ...
    'Position',[30,30,1500,860],'NumberTitle','off');
layout=tiledlayout(fig,3,4,'TileSpacing','compact','Padding','compact');
ax3=nexttile(layout,1,[2,3]);axToF=nexttile(layout,4);axGraph=nexttile(layout,8);
axTimeline=nexttile(layout,9,[1,3]);axStatus=nexttile(layout,12);
styleAxes(ax3);styleAxes(axToF);styleAxes(axGraph);styleAxes(axTimeline);styleAxes(axStatus);

% Main 3-D scene.
hold(ax3,'on');grid(ax3,'on');axis(ax3,'equal');axis(ax3,'vis3d');view(ax3,38,24);
xlim(ax3,[-3.2,3.2]);ylim(ax3,[-3.2,3.2]);zlim(ax3,[-0.1,2.7]);
xlabel(ax3,'World X');ylabel(ax3,'World Y');zlabel(ax3,'World Z');title(ax3,'SkyGraph 3-D Mission','Color','w');
[xg,yg]=meshgrid(-3:0.4:3);surf(ax3,xg,yg,zeros(size(xg)),'FaceColor',[.2,.35,.25],'FaceAlpha',.16,'EdgeAlpha',.12);
plot3(ax3,trajectory(:,1),trajectory(:,2),trajectory(:,3),':','Color',[1,.8,.1],'LineWidth',1.6);
trueTrail=plot3(ax3,nan,nan,nan,'Color',[.1,.75,1],'LineWidth',1.6);
estTrail=plot3(ax3,nan,nan,nan,'--','Color',[1,.5,.1],'LineWidth',1.0);
trueDrone=createDashboardDrone(ax3,[.1,.65,1],false);estimateDrone=createDashboardDrone(ax3,[1,.5,.1],true);
sceneHandles=gobjects(16,1);trackHandles=gobjects(16,1);
for k=1:16
    sceneHandles(k)=plot3(ax3,nan,nan,nan,'o','LineWidth',1.5,'MarkerSize',8);
    trackHandles(k)=plot3(ax3,nan,nan,nan,'o','LineWidth',1.5,'MarkerSize',7);
end
tofWorld=gobjects(10,1);for k=1:10,tofWorld(k)=plot3(ax3,nan,nan,nan,'-','LineWidth',1);end
nominalArrow=quiver3(ax3,0,0,0,0,0,0,0,'Color',[1,.2,.65],'LineWidth',2);
safeArrow=quiver3(ax3,0,0,0,0,0,0,0,'Color',[.15,1,.35],'LineWidth',2.5);

% Body-relative top-down ToF panel.
hold(axToF,'on');grid(axToF,'on');axis(axToF,'equal');xlim(axToF,[-2.1,2.1]);ylim(axToF,[-2.1,2.1]);
title(axToF,'Body-Relative ToF','Color','w');xlabel(axToF,'X forward');ylabel(axToF,'Y left');
[mountOrigins,beamDirections,leftEdges,rightEdges]=dashboardToFGeometry();
conePatch=gobjects(8,1);tofBody=gobjects(10,1);bodyHit=gobjects(10,1);
for k=1:8
    conePatch(k)=patch(axToF,nan,nan,[.1,.8,1],'FaceAlpha',.12,'EdgeAlpha',.35);
end
for k=1:10
    tofBody(k)=plot(axToF,nan,nan,'-','LineWidth',1.2);
    bodyHit(k)=plot(axToF,nan,nan,'.','MarkerSize',12);
end
rectangle(axToF,'Position',[-.11,-.085,.22,.17],'Curvature',.2,'EdgeColor','w','LineWidth',1.5);
upDownText=text(axToF,-2.0,1.95,'','Color','w','VerticalAlignment','top','FontName','Consolas','FontSize',8);

% Graph/GAT panel.
hold(axGraph,'on');grid(axGraph,'on');axis(axGraph,'equal');xlim(axGraph,[-3.1,3.1]);ylim(axGraph,[-3.1,3.1]);
title(axGraph,'Graph + GAT Attention','Color','w');xlabel(axGraph,'Relative X');ylabel(axGraph,'Relative Y');
graphLine=plot(axGraph,nan,nan,'-','Color',[.4,.55,.7],'LineWidth',.7);
attentionLine=plot(axGraph,nan,nan,'-','Color',[1,.85,.1],'LineWidth',2);
graphNodes=gobjects(17,1);for k=1:17,graphNodes(k)=scatter(axGraph,nan,nan,50,[.5,.5,.5],'filled');end

% Timeline panel, normalized for one shared visual scale.
hold(axTimeline,'on');grid(axTimeline,'on');title(axTimeline,'Mission / Safety Timeline','Color','w');xlabel(axTimeline,'Time (s)');ylabel(axTimeline,'Normalized display');
plot(axTimeline,tTrue,min(max(separation,-1),1),'Color',[.2,1,.4],'DisplayName','Tracked sep clipped');
plot(axTimeline,tTrue,gatTopScore(:,1),'Color',[1,.7,.1],'DisplayName','Top GAT score');
stairs(axTimeline,tTrue,intervention,'Color',[1,.2,.2],'DisplayName','CBF intervention');
plot(axTimeline,tTrue,min(packetAge,1),'Color',[.4,.7,1],'DisplayName','Packet age clipped');
yline(axTimeline,0,'--','Color',[.8,.8,.8]);legend(axTimeline,'TextColor','w','Color',[.08,.1,.15],'Location','eastoutside');
timeCursor=xline(axTimeline,tTrue(1),'-w','LineWidth',1.5);

% Text dashboard.
axis(axStatus,'off');statusText=text(axStatus,0.02,.98,'','Units','normalized','VerticalAlignment','top', ...
    'Color','w','FontName','Consolas','FontSize',9,'Interpreter','none');
title(axStatus,'Live Status','Color','w');

setappdata(fig,'DashboardStop',false);setappdata(fig,'DashboardPaused',false);
set(fig,'KeyPressFcn',@dashboardKey);
annotation(fig,'textbox',[.79,.005,.20,.035],'String','Space: pause/resume   Esc: stop', ...
    'Color',[.85,.9,1],'EdgeColor','none','HorizontalAlignment','center');

sampleInterval=median(diff(tTrue));frameStep=max(1,round(1/(options.FrameRate*sampleInterval)));
frameIndices=1:frameStep:numel(tTrue);if frameIndices(end)~=numel(tTrue),frameIndices(end+1)=numel(tTrue);end
wallTimer=tic;startTime=tTrue(1);
for frame=1:numel(frameIndices)
    if ~isgraphics(fig)||getappdata(fig,'DashboardStop'),break;end
    while isgraphics(fig)&&getappdata(fig,'DashboardPaused')
        drawnow;pause(.05);wallTimer=tic;startTime=tTrue(frameIndices(frame));
    end
    i=frameIndices(frame);position=xTrue(i,1:3).';euler=xTrue(i,7:9).';R=dashboardRotation(euler);
    set(trueDrone,'Matrix',dashboardPose(position,euler));set(estimateDrone,'Matrix',dashboardPose(xEst(i,1:3).',xEst(i,7:9).'));
    set(trueTrail,'XData',xTrue(1:i,1),'YData',xTrue(1:i,2),'ZData',xTrue(1:i,3));
    set(estTrail,'XData',xEst(1:i,1),'YData',xEst(1:i,2),'ZData',xEst(1:i,3));

    entities=reshape(scenePosition(i,:).',3,16);tracks=reshape(trackPosition(i,:).',3,16);
    for k=1:16
        if sceneActive(i,k)>.5
            typ=round(sceneType(i,k));color=dashboardTypeColor(typ);marker='o';if typ==2,marker='s';end
            set(sceneHandles(k),'Visible','on','XData',entities(1,k),'YData',entities(2,k),'ZData',entities(3,k),'Color',color,'Marker',marker,'MarkerSize',6+10*sceneRadius(i,k));
        else,set(sceneHandles(k),'Visible','off');end
        if trackActive(i,k)>.5
            score=gatThreat(i,k+1);color=[score,1-score,.1];
            set(trackHandles(k),'Visible','on','XData',tracks(1,k),'YData',tracks(2,k),'ZData',tracks(3,k),'Color',color,'MarkerSize',6+8*score,'LineWidth',1+2*trackConfidence(i,k));
        else,set(trackHandles(k),'Visible','off');end
    end

    % ToF in world and body panels.
    for s=1:10
        lengthValue=tofDistance(i,s);isValid=tofValid(i,s)>.5;if ~isValid,lengthValue=2;end
        color=[.1,.85,1];if ~isValid,color=[.35,.35,.4];end
        originWorld=position+R*mountOrigins(:,s);endWorld=originWorld+R*beamDirections(:,s)*lengthValue;
        set(tofWorld(s),'XData',[originWorld(1),endWorld(1)],'YData',[originWorld(2),endWorld(2)],'ZData',[originWorld(3),endWorld(3)],'Color',color);
        bodyEnd=mountOrigins(:,s)+beamDirections(:,s)*lengthValue;
        set(tofBody(s),'XData',[mountOrigins(1,s),bodyEnd(1)],'YData',[mountOrigins(2,s),bodyEnd(2)],'Color',color);
        set(bodyHit(s),'XData',bodyEnd(1),'YData',bodyEnd(2),'Color',color);
        if s<=8
            leftEnd=mountOrigins(:,s)+leftEdges(:,s)*lengthValue;rightEnd=mountOrigins(:,s)+rightEdges(:,s)*lengthValue;
            set(conePatch(s),'XData',[mountOrigins(1,s),leftEnd(1),rightEnd(1)],'YData',[mountOrigins(2,s),leftEnd(2),rightEnd(2)],'FaceColor',color,'EdgeColor',color);
        end
    end

    set(upDownText,'String',sprintf('UP %.2f m (%d)\nDOWN %.2f m (%d)', ...
        tofDistance(i,9),tofValid(i,9)>.5,tofDistance(i,10),tofValid(i,10)>.5));

    % Dynamic graph panel.
    adjacency=reshape(graphAdjacency(i,:).',17,17)>.5;mask=graphMask(i,:)>.5;
    graphPosition=zeros(2,17);graphPosition(:,2:17)=tracks(1:2,:)-position(1:2);
    [gx,gy]=dashboardEdges(graphPosition,adjacency,mask);set(graphLine,'XData',gx,'YData',gy);
    attention=reshape(gatAttention(i,:).',17,17);target=find(graphId(i,:)==gatTopId(i,1),1);if isempty(target),target=1;end
    attentionAdj=false(17);attentionAdj(target,attention(target,:)>.08)=true;attentionAdj=attentionAdj|attentionAdj.';
    [axx,ayy]=dashboardEdges(graphPosition,attentionAdj,mask);set(attentionLine,'XData',axx,'YData',ayy);
    for node=1:17
        if mask(node)
            score=gatThreat(i,node);color=[score,1-score,.12];if node==1,color=[.1,1,.3];end
            set(graphNodes(node),'XData',graphPosition(1,node),'YData',graphPosition(2,node),'SizeData',45+220*score,'CData',color,'Visible','on');
        else,set(graphNodes(node),'Visible','off');end
    end

    scale=.35;set(nominalArrow,'XData',position(1),'YData',position(2),'ZData',position(3),'UData',scale*nominalAction(i,1),'VData',scale*nominalAction(i,2),'WData',scale*nominalAction(i,3));
    set(safeArrow,'XData',position(1),'YData',position(2),'ZData',position(3),'UData',scale*safeAction(i,1),'VData',scale*safeAction(i,2),'WData',scale*safeAction(i,3));
    set(timeCursor,'Value',tTrue(i));
    policy='WAYPOINT';if policySource(i)>.5,policy='PPO';end
    status=sprintf(['TIME       %6.2f s\nPOLICY     %s\n\nTOP THREAT\n ID        %7.0f\n SCORE     %7.3f\n\n', ...
        'SENSORS\n valid     %2d / 10\n packet age %.2f s\n\nTRACK/GRAPH\n tracks    %2d\n nodes     %2d\n\n', ...
        'CBF\n active    %d\n slack     %.4f\n separation %.3f m\n\n', ...
        'NOMINAL [%.2f %.2f %.2f]\nSAFE    [%.2f %.2f %.2f]'], ...
        tTrue(i),policy,gatTopId(i,1),gatTopScore(i,1),sum(tofValid(i,:)>.5),packetAge(i), ...
        sum(trackActive(i,:)>.5),sum(mask),intervention(i)>.5,slack(i),separation(i), ...
        nominalAction(i,1),nominalAction(i,2),nominalAction(i,3),safeAction(i,1),safeAction(i,2),safeAction(i,3));
    set(statusText,'String',status);
    drawnow;
    targetWall=(tTrue(i)-startTime)/options.PlaybackSpeed;remaining=targetWall-toc(wallTimer);if remaining>0,pause(remaining);end
end
end

function styleAxes(ax)
set(ax,'Color',[.07,.09,.13],'XColor',[.85,.9,1],'YColor',[.85,.9,1],'ZColor',[.85,.9,1],'GridColor',[.4,.48,.58]);
end
function [time,data]=readDash(out,candidates,width)
names=out.who;signal=[];resolved='';for k=1:numel(candidates),name=char(candidates{k});if any(strcmp(names,name)),signal=out.get(name);resolved=name;break;end;end
if isempty(signal),error('Missing dashboard log: %s',strjoin(string(candidates),', '));end
time=signal.Time(:);data=squeeze(signal.Data);if ~(isa(data,'double')||isa(data,'single')),data=double(data);end
if isvector(data),data=data(:);elseif size(data,1)==numel(time);elseif size(data,2)==numel(time),data=data.';else,error('%s dimensions invalid.',resolved);end
if size(data,2)<width,error('%s needs %d columns.',resolved,width);end;data=data(:,1:width);
end
function result=alignDash(time,data,newTime,method)
[ut,index]=unique(time,'stable');result=interp1(ut,data(index,:),newTime,method,'extrap');
end
function root=createDashboardDrone(ax,color,ghost)
root=hgtransform('Parent',ax);style='-';width=5;if ghost,style='--';width=2;end
line('Parent',root,'XData',[-.23,.23],'YData',[0,0],'ZData',[0,0],'Color',color,'LineStyle',style,'LineWidth',width);
line('Parent',root,'XData',[0,0],'YData',[-.23,.23],'ZData',[0,0],'Color',color,'LineStyle',style,'LineWidth',width);
line('Parent',root,'XData',[0,.18],'YData',[0,0],'ZData',[.03,.03],'Color',[1,.1,.1],'LineWidth',2);
end
function T=dashboardPose(p,e),T=eye(4);T(1:3,1:3)=dashboardRotation(e);T(1:3,4)=p;end
function R=dashboardRotation(e),r=e(1);p=e(2);y=e(3);cr=cos(r);sr=sin(r);cp=cos(p);sp=sin(p);cy=cos(y);sy=sin(y);R=[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr;sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr;-sp,cp*sr,cp*cr];end
function color=dashboardTypeColor(type),if type==1,color=[.1,.65,1];elseif type==2,color=[.6,.42,.22];else,color=[1,.25,.15];end;end
function [origins,directions,left,right]=dashboardToFGeometry()
mount=[-22.5,22.5,67.5,112.5,157.5,202.5,247.5,292.5];beam=[-10,10,80,100,170,190,260,280];origins=zeros(3,10);directions=zeros(3,10);left=zeros(3,8);right=zeros(3,8);
for k=1:8,origins(:,k)=[.1*cosd(mount(k));.1*sind(mount(k));-.04];directions(:,k)=[cosd(beam(k));sind(beam(k));0];left(:,k)=[cosd(beam(k)-12.5);sind(beam(k)-12.5);0];right(:,k)=[cosd(beam(k)+12.5);sind(beam(k)+12.5);0];end
origins(:,9)=[0;0;.06];origins(:,10)=[0;0;-.05];directions(:,9)=[0;0;1];directions(:,10)=[0;0;-1];
end
function [x,y]=dashboardEdges(position,adjacency,mask)
x=[];y=[];for i=1:17,for j=i+1:17,if mask(i)&&mask(j)&&adjacency(i,j),x=[x,position(1,i),position(1,j),nan];y=[y,position(2,i),position(2,j),nan];end;end;end
end
function dashboardKey(source,event)
if strcmpi(event.Key,'escape'),setappdata(source,'DashboardStop',true);elseif strcmpi(event.Key,'space'),setappdata(source,'DashboardPaused',~getappdata(source,'DashboardPaused'));end
end
