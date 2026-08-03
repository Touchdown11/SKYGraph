function animateSkyGraphStage14(out,varargin)
%ANIMATESKYGRAPHSTAGE14 Complete post-simulation SkyGraph playback.

p=inputParser;addParameter(p,'PlaybackSpeed',2,@(x)isnumeric(x)&&x>0);
addParameter(p,'FrameRate',25,@(x)isnumeric(x)&&x>=5);
addParameter(p,'CameraMode','fixed',@(x)ischar(x)||isstring(x));parse(p,varargin{:});
opt=p.Results;cameraMode=lower(string(opt.CameraMode));
if ~any(cameraMode==["fixed","chase","top"]),error('CameraMode: fixed, chase or top.');end

[tTrue,xTrue]=readAny({'true_state14_log','true_state8_log','true_state4w_log'},16);
[tEst,xEst]=readAny({'estimated_state14_log','estimated_state4w_log','estimated_state3_log'},12);
[tTraj,traj]=readAny({'trajectory14_log','trajectory4w_log'},10);
[tScen,scenPos]=readAny({'scenario8_position_log'},48);
[~,scenRadius]=readAny({'scenario8_radius_log'},16);[~,scenType]=readAny({'scenario8_type_log'},16);[~,scenActive]=readAny({'scenario8_active_log'},16);
[tToF,tofDistance]=readAny({'tof8_distance_log'},10);[~,tofValid]=readAny({'tof8_valid_log'},10);[~,packetAge]=readAny({'tof8_packet_age_log'},1);
[tTrack,trackPos]=readAny({'track9_position_log'},48);[~,trackId]=readAny({'track9_id_log'},16);[~,trackType]=readAny({'track9_type_log'},16);[~,trackActive]=readAny({'track9_active_log'},16);[~,trackConfidence]=readAny({'track9_confidence_log'},16);
[tGraph,graphMask]=readAny({'graph10_mask_log'},17);[~,graphId]=readAny({'graph10_id_log'},17);[~,adjVec]=readAny({'graph10_adjacency_log'},289);
[tGAT,gatThreat]=readAny({'gat12_threat_log'},17);[~,gatAttention]=readAny({'gat12_attention_log'},289);[~,gatTopId]=readAny({'gat12_top_id_log'},5);[~,gatTopScore]=readAny({'gat12_top_score_log'},5);
[tPolicy,selectedNominal]=readAny({'ppo13_selected_log'},3);[~,policySource]=readAny({'ppo13_source_log'},1);
[tCBF,safeAcceleration]=readAny({'cbf11_safe_log'},3);[~,intervention]=readAny({'cbf11_intervention_log'},1);[~,slack]=readAny({'cbf11_slack_log'},1);[~,minimumSeparation]=readAny({'cbf11_separation_log'},1);

% Align all discrete signals to the true-state playback clock.
xEst=interp14(tEst,xEst,tTrue,'linear');traj=interp14(tTraj,traj,tTrue,'linear');
scenPos=interp14(tScen,scenPos,tTrue,'previous');scenRadius=interp14(tScen,scenRadius,tTrue,'previous');scenType=interp14(tScen,scenType,tTrue,'previous');scenActive=interp14(tScen,scenActive,tTrue,'previous');
tofDistance=interp14(tToF,tofDistance,tTrue,'previous');tofValid=interp14(tToF,tofValid,tTrue,'previous');packetAge=interp14(tToF,packetAge,tTrue,'previous');
trackPos=interp14(tTrack,trackPos,tTrue,'previous');trackId=interp14(tTrack,trackId,tTrue,'previous');trackType=interp14(tTrack,trackType,tTrue,'previous');trackActive=interp14(tTrack,trackActive,tTrue,'previous');trackConfidence=interp14(tTrack,trackConfidence,tTrue,'previous');
graphMask=interp14(tGraph,graphMask,tTrue,'previous');graphId=interp14(tGraph,graphId,tTrue,'previous');adjVec=interp14(tGraph,adjVec,tTrue,'previous');
gatThreat=interp14(tGAT,gatThreat,tTrue,'previous');gatAttention=interp14(tGAT,gatAttention,tTrue,'previous');gatTopId=interp14(tGAT,gatTopId,tTrue,'previous');gatTopScore=interp14(tGAT,gatTopScore,tTrue,'previous');
selectedNominal=interp14(tPolicy,selectedNominal,tTrue,'previous');policySource=interp14(tPolicy,policySource,tTrue,'previous');
safeAcceleration=interp14(tCBF,safeAcceleration,tTrue,'previous');intervention=interp14(tCBF,intervention,tTrue,'previous');slack=interp14(tCBF,slack,tTrue,'previous');minimumSeparation=interp14(tCBF,minimumSeparation,tTrue,'previous');

fig=figure('Name','Stage 14 Complete SkyGraph Visualization','Color',[0.04,0.06,0.10],'Position',[50,50,1400,820]);
ax=axes('Parent',fig,'Position',[0.04,0.07,0.74,0.88]);hold(ax,'on');grid(ax,'on');axis(ax,'equal');axis(ax,'vis3d');
set(ax,'Color',[0.08,0.10,0.15],'XColor','w','YColor','w','ZColor','w');
xlabel(ax,'World X');ylabel(ax,'World Y');zlabel(ax,'World Z');xlim(ax,[-3.2,3.2]);ylim(ax,[-3.2,3.2]);zlim(ax,[-0.1,2.7]);
[xg,yg]=meshgrid(-3:0.3:3);surf(ax,xg,yg,zeros(size(xg)),'FaceAlpha',0.18,'EdgeAlpha',0.18,'FaceColor',[0.2,0.35,0.25]);
plot3(ax,traj(:,1),traj(:,2),traj(:,3),':','Color',[1,0.8,0.1],'LineWidth',1.8);
trueTrail=plot3(ax,nan,nan,nan,'Color',[0.1,0.75,1],'LineWidth',1.8);
estTrail=plot3(ax,nan,nan,nan,'--','Color',[1,0.5,0.1],'LineWidth',1.1);
[trueDrone,rotorHandles]=createDrone14(ax,[0.08,0.48,0.82],false);
[estimateDrone,~]=createDrone14(ax,[1,0.5,0.1],true);

scenarioHandles=gobjects(16,1);trackHandles=gobjects(16,1);
for k=1:16
    scenarioHandles(k)=plot3(ax,nan,nan,nan,'o','MarkerSize',9,'LineWidth',1.5);
    trackHandles(k)=plot3(ax,nan,nan,nan,'o','MarkerSize',7,'LineWidth',1.5,'MarkerFaceColor','none');
end

graphEdges=plot3(ax,nan,nan,nan,'-','Color',[0.5,0.65,0.8],'LineWidth',0.7);
attentionEdges=plot3(ax,nan,nan,nan,'-','Color',[1,0.9,0.1],'LineWidth',2.0);
nominalArrow=quiver3(ax,0,0,0,0,0,0,0,'Color',[1,0.25,0.65],'LineWidth',2,'MaxHeadSize',0.5);
safeArrow=quiver3(ax,0,0,0,0,0,0,0,'Color',[0.2,1,0.4],'LineWidth',2.5,'MaxHeadSize',0.5);

tofCenter=gobjects(10,1);tofPatch=gobjects(8,1);hitMarker=gobjects(10,1);
for k=1:10
    tofCenter(k)=plot3(ax,nan,nan,nan,'-','LineWidth',1.1);
    hitMarker(k)=plot3(ax,nan,nan,nan,'.','MarkerSize',13);
end
for k=1:8,tofPatch(k)=patch(ax,nan,nan,nan,[0.1,0.8,1],'FaceAlpha',0.08,'EdgeAlpha',0.25);end

status=annotation(fig,'textbox',[0.80,0.45,0.19,0.47],'String','', ...
    'Color','w','BackgroundColor',[0.08,0.10,0.15],'EdgeColor',[0.4,0.55,0.7], ...
    'FontName','Consolas','FontSize',10,'Interpreter','none');
annotation(fig,'textbox',[0.80,0.31,0.19,0.10], ...
    'String',sprintf('Blue true | Orange estimate\nGold desired | Cyan ToF\nYellow attention | Esc stops'), ...
    'Color',[0.85,0.9,1],'BackgroundColor',[0.08,0.10,0.15]);
setappdata(fig,'stop14',false);set(fig,'KeyPressFcn',@(src,event)setStop(src,event));
if cameraMode=="fixed",view(ax,38,24);elseif cameraMode=="top",view(ax,0,90);else,view(ax,38,20);end

sampleInterval=median(diff(tTrue));step=max(1,round(1/(opt.FrameRate*sampleInterval)));indices=1:step:numel(tTrue);if indices(end)~=numel(tTrue),indices(end+1)=numel(tTrue);end
rotorAngle=zeros(4,1);previous=indices(1);timer=tic;t0=tTrue(1);
for frame=1:numel(indices)
    if ~isgraphics(fig)||getappdata(fig,'stop14'),break;end
    i=indices(frame);position=xTrue(i,1:3).';attitude=xTrue(i,7:9).';R=rotation14(attitude);
    set(trueDrone,'Matrix',pose14(position,attitude));set(estimateDrone,'Matrix',pose14(xEst(i,1:3).',xEst(i,7:9).'));
    dt=tTrue(i)-tTrue(previous);rotorAngle=rotorAngle+[1;-1;1;-1].*xTrue(i,13:16).'*dt;previous=i;
    rotorPositions=[.23,0,-.23,0;0,.23,0,-.23;.025,.025,.025,.025];
    for m=1:4,set(rotorHandles(m),'Matrix',translation14(rotorPositions(:,m))*rotationZ14(rotorAngle(m)));end
    set(trueTrail,'XData',xTrue(1:i,1),'YData',xTrue(1:i,2),'ZData',xTrue(1:i,3));set(estTrail,'XData',xEst(1:i,1),'YData',xEst(1:i,2),'ZData',xEst(1:i,3));

    entityPosition=reshape(scenPos(i,:).',3,16);trackedPosition=reshape(trackPos(i,:).',3,16);
    for k=1:16
        if scenActive(i,k)>.5
            typ=round(scenType(i,k));color=typeColor14(typ);marker='o';if typ==2,marker='s';end
            set(scenarioHandles(k),'XData',entityPosition(1,k),'YData',entityPosition(2,k),'ZData',entityPosition(3,k), ...
                'Visible','on','Color',color,'Marker',marker,'MarkerSize',6+10*scenRadius(i,k));
        else,set(scenarioHandles(k),'Visible','off');end
        if trackActive(i,k)>.5
            node=k+1;score=gatThreat(i,node);color=[score,1-score,0.15];
            set(trackHandles(k),'XData',trackedPosition(1,k),'YData',trackedPosition(2,k),'ZData',trackedPosition(3,k), ...
                'Visible','on','Color',color,'MarkerSize',6+8*score,'LineWidth',1+2*trackConfidence(i,k));
        else,set(trackHandles(k),'Visible','off');end
    end

    adjacency=reshape(adjVec(i,:).',17,17)>.5;mask=graphMask(i,:)>.5;
    nodePosition=[position,trackedPosition];[gx,gy,gz]=edgeSegments14(nodePosition,adjacency,mask);set(graphEdges,'XData',gx,'YData',gy,'ZData',gz);
    attention=reshape(gatAttention(i,:).',17,17);topId=gatTopId(i,1);target=find(graphId(i,:)==topId,1);if isempty(target),target=1;end
    attentionAdj=attention(target,:)>0.08;attentionMatrix=false(17);attentionMatrix(target,attentionAdj)=true;attentionMatrix=attentionMatrix|attentionMatrix.';[axv,ayv,azv]=edgeSegments14(nodePosition,attentionMatrix,mask);set(attentionEdges,'XData',axv,'YData',ayv,'ZData',azv);

    [origins,directions,leftDirections,rightDirections]=tofGeometry14();
    for s=1:10
        originWorld=position+R*origins(:,s);lengthValue=tofDistance(i,s);if tofValid(i,s)<.5,lengthValue=2;end
        endWorld=originWorld+R*directions(:,s)*lengthValue;color=[0.1,0.85,1];if tofValid(i,s)<.5,color=[0.35,0.35,0.4];end
        set(tofCenter(s),'XData',[originWorld(1),endWorld(1)],'YData',[originWorld(2),endWorld(2)],'ZData',[originWorld(3),endWorld(3)],'Color',color);
        set(hitMarker(s),'XData',endWorld(1),'YData',endWorld(2),'ZData',endWorld(3),'Color',color);
        if s<=8
            leftEnd=originWorld+R*leftDirections(:,s)*lengthValue;rightEnd=originWorld+R*rightDirections(:,s)*lengthValue;
            set(tofPatch(s),'XData',[originWorld(1),leftEnd(1),rightEnd(1)],'YData',[originWorld(2),leftEnd(2),rightEnd(2)],'ZData',[originWorld(3),leftEnd(3),rightEnd(3)],'FaceColor',color,'EdgeColor',color);
        end
    end

    scale=.35;set(nominalArrow,'XData',position(1),'YData',position(2),'ZData',position(3),'UData',scale*selectedNominal(i,1),'VData',scale*selectedNominal(i,2),'WData',scale*selectedNominal(i,3));
    set(safeArrow,'XData',position(1),'YData',position(2),'ZData',position(3),'UData',scale*safeAcceleration(i,1),'VData',scale*safeAcceleration(i,2),'WData',scale*safeAcceleration(i,3));
    policy='WAYPOINT';if policySource(i)>.5,policy='PPO';end
    text=sprintf(['TIME %6.2f s\nPOLICY %s\n\nTOP THREAT\nID %7.0f  SCORE %.3f\n\n', ...
        'TOF VALID %2d/10  AGE %.2f s\nTRACKS %2d  GRAPH NODES %2d\n\n', ...
        'CBF ACTIVE %d\nSLACK %.4f\nMIN SEP %.3f m\n\n', ...
        'NOM [%.2f %.2f %.2f]\nSAFE[%.2f %.2f %.2f]'],tTrue(i),policy,gatTopId(i,1),gatTopScore(i,1), ...
        sum(tofValid(i,:)>.5),packetAge(i),sum(trackActive(i,:)>.5),sum(mask),intervention(i)>.5,slack(i),minimumSeparation(i), ...
        selectedNominal(i,1),selectedNominal(i,2),selectedNominal(i,3),safeAcceleration(i,1),safeAcceleration(i,2),safeAcceleration(i,3));
    set(status,'String',text);
    if cameraMode=="chase",yaw=attitude(3);campos(ax,(position+[-4*cos(yaw);-4*sin(yaw);2]).');camtarget(ax,(position+[0;0;.15]).');camup(ax,[0,0,1]);end
    drawnow;remaining=(tTrue(i)-t0)/opt.PlaybackSpeed-toc(timer);if remaining>0,pause(remaining);end
end

    function [time,data]=readAny(candidates,width)
        names=out.who;signal=[];
        for kk=1:numel(candidates),name=char(candidates{kk});if any(strcmp(names,name)),signal=out.get(name);break;end;end
        if isempty(signal),error('Missing Stage 14 log: %s',strjoin(string(candidates),', '));end
        time=signal.Time(:);data=squeeze(signal.Data);if isvector(data),data=data(:);elseif size(data,1)==numel(time);elseif size(data,2)==numel(time),data=data.';else,error('Log dimensions invalid.');end
        if size(data,2)<width,error('Log needs %d columns.',width);end;data=data(:,1:width);
    end
end

function result=interp14(time,data,newTime,method)
% interp1 does not accept logical sample values. Stage 8-13 masks and
% flags are often logical, so convert them to double before sample/hold.
if ~(isa(data,'double') || isa(data,'single'))
    data=double(data);
end
[uniqueTime,index]=unique(time,'stable');
result=interp1(uniqueTime,data(index,:),newTime,method,'extrap');
end
function [root,rotors]=createDrone14(ax,color,ghost)
root=hgtransform('Parent',ax);style='-';width=5;if ghost,style='--';width=2;end
line('Parent',root,'XData',[-.23,.23],'YData',[0,0],'ZData',[0,0],'Color',color,'LineStyle',style,'LineWidth',width);
line('Parent',root,'XData',[0,0],'YData',[-.23,.23],'ZData',[0,0],'Color',color,'LineStyle',style,'LineWidth',width);
line('Parent',root,'XData',[0,.18],'YData',[0,0],'ZData',[.03,.03],'Color',[1,.15,.1],'LineWidth',2);
rotors=gobjects(4,1);for k=1:4,rotors(k)=hgtransform('Parent',root);line('Parent',rotors(k),'XData',[-.09,.09],'YData',[0,0],'ZData',[0,0],'Color',color,'LineWidth',2);end
end
function T=pose14(p,e),T=eye(4);T(1:3,1:3)=rotation14(e);T(1:3,4)=p;end
function R=rotation14(e),r=e(1);p=e(2);y=e(3);cr=cos(r);sr=sin(r);cp=cos(p);sp=sin(p);cy=cos(y);sy=sin(y);R=[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr;sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr;-sp,cp*sr,cp*cr];end
function T=translation14(p),T=eye(4);T(1:3,4)=p;end
function T=rotationZ14(a),T=[cos(a),-sin(a),0,0;sin(a),cos(a),0,0;0,0,1,0;0,0,0,1];end
function color=typeColor14(type),if type==1,color=[.1,.65,1];elseif type==2,color=[.6,.42,.22];else,color=[1,.25,.15];end;end
function [x,y,z]=edgeSegments14(position,adjacency,mask)
x=[];y=[];z=[];for i=1:17,for j=i+1:17,if mask(i)&&mask(j)&&adjacency(i,j),x=[x,position(1,i),position(1,j),nan];y=[y,position(2,i),position(2,j),nan];z=[z,position(3,i),position(3,j),nan];end;end;end
end
function [origins,directions,left,right]=tofGeometry14()
mount=[-22.5,22.5,67.5,112.5,157.5,202.5,247.5,292.5];beam=[-10,10,80,100,170,190,260,280];origins=zeros(3,10);directions=zeros(3,10);left=zeros(3,8);right=zeros(3,8);
for k=1:8,origins(:,k)=[.1*cosd(mount(k));.1*sind(mount(k));-.04];directions(:,k)=[cosd(beam(k));sind(beam(k));0];left(:,k)=[cosd(beam(k)-12.5);sind(beam(k)-12.5);0];right(:,k)=[cosd(beam(k)+12.5);sind(beam(k)+12.5);0];end
origins(:,9)=[0;0;.06];origins(:,10)=[0;0;-.05];directions(:,9)=[0;0;1];directions(:,10)=[0;0;-1];
end
function setStop(source,event),if strcmpi(event.Key,'escape'),setappdata(source,'stop14',true);end;end
