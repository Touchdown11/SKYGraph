function animateMultiEgoSkyGraphDashboardStage14(output,varargin)
%ANIMATEMULTIEGOSKYGRAPHDASHBOARDSTAGE14 Stage-14 style multi-ego playback.
% Input is the output returned by runMultiEgoSimulationStage16.
p=inputParser;addParameter(p,'PlaybackSpeed',2,@(x)isnumeric(x)&&isscalar(x)&&x>0);addParameter(p,'FrameRate',20,@(x)isnumeric(x)&&isscalar(x)&&x>=5);parse(p,varargin{:});opt=p.Results;
required={'Time','Position','Velocity','Goal','MinimumPairwiseSeparation','CBFIntervention','GraphNodes','GraphEdges','FinalGoalError'};
for k=1:numel(required),if ~isfield(output,required{k}),error('Stage14:MultiEgoOutput','Missing multi-ego field %s.',required{k});end,end
T=output.Time(:);N=size(output.Position,2);K=numel(T); colors=lines(N);
fig=figure('Name','SkyGraph Stage 14 Multi-Ego Mission Dashboard','Color',[.035,.05,.08],'Position',[30,30,1500,860],'NumberTitle','off');
layout=tiledlayout(fig,3,4,'TileSpacing','compact','Padding','compact');axWorld=nexttile(layout,1,[2,3]);axSep=nexttile(layout,4);axCBF=nexttile(layout,8);axTimeline=nexttile(layout,9,[1,3]);axStatus=nexttile(layout,12);
axesList=[axWorld axSep axCBF axTimeline axStatus];for ax=axesList,set(ax,'Color',[.07,.09,.13],'XColor',[.85,.9,1],'YColor',[.85,.9,1],'ZColor',[.85,.9,1],'GridColor',[.4,.48,.58]);end
hold(axWorld,'on');grid(axWorld,'on');axis(axWorld,'equal');axis(axWorld,'vis3d');view(axWorld,38,24);xlim(axWorld,[-3.2 3.2]);ylim(axWorld,[-3.2 3.2]);zlim(axWorld,[-.1 2.7]);xlabel(axWorld,'World X');ylabel(axWorld,'World Y');zlabel(axWorld,'World Z');title(axWorld,'Stage 14 — Multi-Ego Shared World','Color','w');[xg,yg]=meshgrid(-3:.4:3);surf(axWorld,xg,yg,zeros(size(xg)),'FaceColor',[.2 .35 .25],'FaceAlpha',.16,'EdgeAlpha',.12);
trail=gobjects(N,1);drone=gobjects(N,1);goalMarker=gobjects(N,1);label=gobjects(N,1);
for ego=1:N
    p0=squeeze(output.Position(:,ego,1));g=output.Goal(:,ego);trail(ego)=plot3(axWorld,nan,nan,nan,'Color',colors(ego,:),'LineWidth',1.8);drone(ego)=makeDrone16(axWorld,colors(ego,:));goalMarker(ego)=plot3(axWorld,g(1),g(2),g(3),'x','Color',colors(ego,:),'MarkerSize',10,'LineWidth',2);label(ego)=text(axWorld,p0(1),p0(2),p0(3)+.18,sprintf('EGO %d',ego),'Color',colors(ego,:),'FontWeight','bold');
end
hold(axSep,'on');grid(axSep,'on');title(axSep,'Pairwise Safety','Color','w');plot(axSep,T,output.MinimumPairwiseSeparation,'Color',[.2 1 .4],'LineWidth',1.5);yline(axSep,0,'r--','Collision');yline(axSep,.18,'--','Color',[.95 .6 .1],'Label','margin');xlabel(axSep,'Time (s)');ylabel(axSep,'Surface separation (m)');cursor1=xline(axSep,T(1),'-w');
hold(axCBF,'on');grid(axCBF,'on');title(axCBF,'CBF Intervention by Ego','Color','w');for ego=1:N,stairs(axCBF,T,double(output.CBFIntervention(ego,:))+(ego-1)*1.2,'Color',colors(ego,:),'LineWidth',1.3,'DisplayName',sprintf('Ego %d',ego));end;ylim(axCBF,[-.2 max(1,N*1.2)]);xlabel(axCBF,'Time (s)');ylabel(axCBF,'Active lane');legend(axCBF,'TextColor','w','Color',[.08 .1 .15],'Location','best');cursor2=xline(axCBF,T(1),'-w');
hold(axTimeline,'on');grid(axTimeline,'on');title(axTimeline,'Multi-Ego Graph / Mission Timeline','Color','w');for ego=1:N,plot(axTimeline,T,output.GraphNodes(ego,:),'Color',colors(ego,:),'LineWidth',1.1,'DisplayName',sprintf('Ego %d graph nodes',ego));end;plot(axTimeline,T,output.MinimumPairwiseSeparation,'w','LineWidth',1.8,'DisplayName','Min separation');xlabel(axTimeline,'Time (s)');ylabel(axTimeline,'Nodes / metres');legend(axTimeline,'TextColor','w','Color',[.08 .1 .15],'Location','eastoutside');cursor3=xline(axTimeline,T(1),'-w');axis(axStatus,'off');title(axStatus,'Fleet Status','Color','w');status=text(axStatus,.02,.98,'','Units','normalized','VerticalAlignment','top','Color','w','FontName','Consolas','FontSize',9,'Interpreter','none');
setappdata(fig,'stop',false);setappdata(fig,'pause',false);set(fig,'KeyPressFcn',@key);annotation(fig,'textbox',[.79,.005,.2,.035],'String','Space: pause/resume   Esc: stop','Color',[.85 .9 1],'EdgeColor','none','HorizontalAlignment','center');
step=max(1,round(1/(opt.FrameRate*median(diff(T)))));indices=1:step:K;if indices(end)~=K,indices(end+1)=K;end;timer=tic;t0=T(1);
for f=1:numel(indices)
    if ~isgraphics(fig)||getappdata(fig,'stop'),break,end
    while isgraphics(fig)&&getappdata(fig,'pause'),drawnow;pause(.05);timer=tic;t0=T(indices(f));end
    i=indices(f);
    for ego=1:N
        pos=squeeze(output.Position(:,ego,i));vel=squeeze(output.Velocity(:,ego,i));yaw=atan2(vel(2),vel(1));set(drone(ego),'Matrix',pose16(pos,yaw));set(trail(ego),'XData',squeeze(output.Position(1,ego,1:i)),'YData',squeeze(output.Position(2,ego,1:i)),'ZData',squeeze(output.Position(3,ego,1:i)));set(label(ego),'Position',pos+[0;0;.18]);
    end
    set(cursor1,'Value',T(i));set(cursor2,'Value',T(i));set(cursor3,'Value',T(i));active=sum(output.CBFIntervention(:,i));statusText=sprintf('TIME       %6.2f s\nEGOS       %d\n\nMIN PAIR SEP  %.3f m\nCOLLISION     %d\nCBF ACTIVE    %d\n\n',T(i),N,output.MinimumPairwiseSeparation(i),output.Collision(i),active);
    for ego=1:N,statusText=sprintf('%sEGO %d  nodes %2d  edges %2d  goal err %.2f\n',statusText,ego,output.GraphNodes(ego,i),output.GraphEdges(ego,i),norm(squeeze(output.Position(:,ego,i))-output.Goal(:,ego)));end
    set(status,'String',statusText);drawnow;remaining=(T(i)-t0)/opt.PlaybackSpeed-toc(timer);if remaining>0,pause(remaining);end
end
    function key(src,event)
        if strcmpi(event.Key,'escape'),setappdata(src,'stop',true);elseif strcmpi(event.Key,'space'),setappdata(src,'pause',~getappdata(src,'pause'));end
    end
end
function root=makeDrone16(ax,color)
root=hgtransform('Parent',ax);line('Parent',root,'XData',[-.23 .23],'YData',[0 0],'ZData',[0 0],'Color',color,'LineWidth',5);line('Parent',root,'XData',[0 0],'YData',[-.23 .23],'ZData',[0 0],'Color',color,'LineWidth',5);line('Parent',root,'XData',[0 .18],'YData',[0 0],'ZData',[.03 .03],'Color',[1 .15 .1],'LineWidth',2);
end
function T=pose16(position,yaw),T=[cos(yaw),-sin(yaw),0,position(1);sin(yaw),cos(yaw),0,position(2);0,0,1,position(3);0,0,0,1];end
