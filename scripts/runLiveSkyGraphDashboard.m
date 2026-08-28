function capture = runLiveSkyGraphDashboard(localPort)
%RUNLIVESKYGRAPHDASHBOARD Receive ESP32 JSON UDP and visualize ten beams.
% Press Esc or close the figure to finish and save a MAT capture.

if nargin<1,localPort=5005;end
[receiver,receiverMode]=createReceiver(localPort);
cleanup=onCleanup(@()releaseReceiver(receiver,receiverMode)); %#ok

fig=figure('Name','Live SkyGraph ESP32 ToF Dashboard','Color',[.04,.06,.1], ...
    'Position',[80,80,1250,720],'NumberTitle','off');
layout=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
axMap=nexttile(layout,1);axBar=nexttile(layout,2);axHistory=nexttile(layout,3);axStatus=nexttile(layout,4);
styleLiveAxes(axMap);styleLiveAxes(axBar);styleLiveAxes(axHistory);styleLiveAxes(axStatus);

% Body-relative cone panel.
hold(axMap,'on');grid(axMap,'on');axis(axMap,'equal');xlim(axMap,[-2.1,2.1]);ylim(axMap,[-2.1,2.1]);
xlabel(axMap,'Body X forward (m)');ylabel(axMap,'Body Y left (m)');title(axMap,'Real ToF Sector Map','Color','w');
[mount,beam,leftEdge,rightEdge]=liveGeometry();
cone=gobjects(8,1);ray=gobjects(10,1);hit=gobjects(10,1);
for sensor=1:8
    cone(sensor)=patch(axMap,nan,nan,[.2,.8,1],'FaceAlpha',.12,'EdgeAlpha',.4);
end
for sensor=1:10
    ray(sensor)=plot(axMap,nan,nan,'-','LineWidth',1.5);
    hit(sensor)=plot(axMap,nan,nan,'.','MarkerSize',15);
end
rectangle(axMap,'Position',[-.11,-.085,.22,.17],'Curvature',.2,'EdgeColor','w','LineWidth',1.5);
verticalText=text(axMap,-2,2,'','Color','w','VerticalAlignment','top','FontName','Consolas');

% Ten-sensor bar panel.
hold(axBar,'on');grid(axBar,'on');title(axBar,'Individual Sensor Ranges','Color','w');
barHandle=bar(axBar,0:9,2*ones(1,10),'FaceColor','flat');ylim(axBar,[0,2.1]);
xlabel(axBar,'Sensor ID');ylabel(axBar,'Range (m)');xticks(axBar,0:9);

% Six-direction history.
hold(axHistory,'on');grid(axHistory,'on');title(axHistory,'Nearest Direction Ranges','Color','w');
colors=lines(6);historyLines=gobjects(6,1);
for k=1:6,historyLines(k)=plot(axHistory,nan,nan,'Color',colors(k,:),'LineWidth',1.2);end
xlabel(axHistory,'Host elapsed time (s)');ylabel(axHistory,'Range (m)');ylim(axHistory,[0,2.1]);
legend(axHistory,{'Front','Left','Rear','Right','Up','Down'},'TextColor','w','Color',[.08,.1,.15]);

axis(axStatus,'off');statusText=text(axStatus,.02,.98,'Waiting for UDP packets...', ...
    'Units','normalized','VerticalAlignment','top','Color','w', ...
    'FontName','Consolas','FontSize',10,'Interpreter','none');
setappdata(fig,'StopLiveSkyGraph',false);set(fig,'KeyPressFcn',@stopKey);

capture=struct('time',zeros(0,1),'distanceM',zeros(0,10), ...
    'valid',false(0,10),'status',zeros(0,10),'sequence',zeros(0,1), ...
    'deviceTimeMs',zeros(0,1),'scanTimeMs',zeros(0,1), ...
    'arrivalInterval',zeros(0,1),'droppedTotal',zeros(0,1));
sessionClock=tic;lastPacketClock=[];lastArrival=nan;lastSequence=nan;
droppedTotal=0;duplicateCount=0;parseErrors=0;sender="";
latestDistance=2*ones(10,1);latestValid=false(10,1);latestStatus=2*ones(10,1);
historyTime=zeros(0,1);historyDirection=zeros(0,6);

while isgraphics(fig)&&~getappdata(fig,'StopLiveSkyGraph')
    [payloads,senders]=receivePayloads(receiver,receiverMode);
    for packet=1:numel(payloads)
        [frame,ok,~]=parseSkyGraphJSON(payloads{packet});
        if ~ok,parseErrors=parseErrors+1;continue;end
        arrival=toc(sessionClock);sender=senders(packet);
        if isnan(lastArrival),interval=0;else,interval=arrival-lastArrival;end
        if ~isnan(lastSequence)
            difference=mod(frame.sequence-lastSequence,2^32);
            if difference==0
                duplicateCount=duplicateCount+1;
            elseif difference>1&&difference<1e6
                droppedTotal=droppedTotal+difference-1;
            end
        end
        lastSequence=frame.sequence;lastArrival=arrival;lastPacketClock=tic;
        latestDistance=frame.distanceM;latestValid=frame.valid;latestStatus=frame.status;
        capture.time(end+1,1)=arrival;capture.distanceM(end+1,:)=frame.distanceM.';
        capture.valid(end+1,:)=frame.valid.';capture.status(end+1,:)=frame.status.';
        capture.sequence(end+1,1)=frame.sequence;capture.deviceTimeMs(end+1,1)=frame.deviceTimeMs;
        capture.scanTimeMs(end+1,1)=frame.scanTimeMs;capture.arrivalInterval(end+1,1)=interval;
        capture.droppedTotal(end+1,1)=droppedTotal;
    end

    if isempty(lastPacketClock),age=inf;else,age=toc(lastPacketClock);end
    displayValid=latestValid & (age<=0.15);
    directionDistance=2*ones(6,1);directionValid=false(6,1);
    pairs=[1,2;3,4;5,6;7,8;9,9;10,10];
    for direction=1:6
        indices=pairs(direction,1):pairs(direction,2);
        candidates=latestDistance(indices);candidateValid=displayValid(indices);
        if any(candidateValid)
            directionDistance(direction)=min(candidates(candidateValid));directionValid(direction)=true;
        end
    end

    for sensor=1:10
        lengthValue=latestDistance(sensor);color=[.1,.85,1];
        if ~displayValid(sensor),lengthValue=2;color=[.35,.35,.4];end
        endpoint=mount(:,sensor)+beam(:,sensor)*lengthValue;
        set(ray(sensor),'XData',[mount(1,sensor),endpoint(1)],'YData',[mount(2,sensor),endpoint(2)],'Color',color);
        set(hit(sensor),'XData',endpoint(1),'YData',endpoint(2),'Color',color);
        if sensor<=8
            leftPoint=mount(:,sensor)+leftEdge(:,sensor)*lengthValue;
            rightPoint=mount(:,sensor)+rightEdge(:,sensor)*lengthValue;
            set(cone(sensor),'XData',[mount(1,sensor),leftPoint(1),rightPoint(1)], ...
                'YData',[mount(2,sensor),leftPoint(2),rightPoint(2)],'FaceColor',color,'EdgeColor',color);
        end
    end
    set(verticalText,'String',sprintf('UP %.2f m (%d)\nDOWN %.2f m (%d)', ...
        latestDistance(9),displayValid(9),latestDistance(10),displayValid(10)));
    set(barHandle,'YData',latestDistance.');barColors=repmat([.35,.35,.4],10,1);
    barColors(displayValid,:) = repmat([.1,.75,1],sum(displayValid),1);set(barHandle,'CData',barColors);

    if isfinite(age)&&~isempty(lastPacketClock)
        now=toc(sessionClock);historyTime(end+1,1)=now;row=directionDistance.';row(~directionValid)=nan;
        historyDirection(end+1,:)=row;keep=historyTime>=now-60;historyTime=historyTime(keep);historyDirection=historyDirection(keep,:);
        for k=1:6,set(historyLines(k),'XData',historyTime,'YData',historyDirection(:,k));end
        if now>60,xlim(axHistory,[now-60,now]);end
    end

    alive=sum(latestStatus~=2);validCount=sum(displayValid);
    if isfinite(age),ageText=sprintf('%.3f s',age);else,ageText='NO PACKET';end
    rate=0;if size(capture.time,1)>=2,window=max(1,size(capture.time,1)-20);rate=1/max(mean(diff(capture.time(window:end))),eps);end
    message=sprintf(['UDP PORT %d\nRECEIVER %s\nSENDER %s\n\n', ...
        'SEQUENCE %.0f\nPACKET AGE %s\nRATE %.1f Hz\nSCAN TIME %.0f ms\n\n', ...
        'VALID %d / 10\nSENSORS ALIVE %d / 10\nDROPPED %.0f\nDUPLICATES %.0f\nPARSE ERRORS %.0f\n\n', ...
        'Press Esc or close window to stop.'],localPort,receiverMode,sender,lastSequence,ageText,rate, ...
        valueOrZero(capture.scanTimeMs),validCount,alive,droppedTotal,duplicateCount,parseErrors);
    set(statusText,'String',message);drawnow limitrate;pause(.01);
end

if ~isempty(capture.time)
    filename="real_tof_capture_"+string(datetime('now','Format','yyyyMMdd_HHmmss'))+".mat";
    save(filename,'capture');fprintf('Saved real ToF capture: %s\n',filename);
else
    fprintf('No valid packets were captured.\n');
end
end

function [receiver,mode]=createReceiver(port)
if exist('udpport','file')==2
    receiver=udpport("datagram","IPV4","LocalPort",port);mode="udpport";
elseif exist('dsp.UDPReceiver','class')==8
    receiver=dsp.UDPReceiver('LocalIPPort',port,'RemoteIPAddress','0.0.0.0', ...
        'MessageDataType','uint8','MaximumMessageLength',512,'ReceiveBufferSize',65536);mode="dsp.UDPReceiver";
else
    error('Neither udpport nor dsp.UDPReceiver is available.');
end
end
function [payloads,senders]=receivePayloads(receiver,mode)
payloads={};senders=strings(0,1);
if mode=="udpport"
    count=receiver.NumDatagramsAvailable;if count>0
        datagrams=read(receiver,count,"string");
        for k=1:numel(datagrams),payloads{end+1}=char(datagrams(k).Data);senders(end+1,1)=string(datagrams(k).SenderAddress);end %#ok
    end
else
    data=receiver();if ~isempty(data),data=uint8(data(:).');data=data(data~=0);payloads={char(data)};senders="UDP peer";end
end
end
function releaseReceiver(receiver,mode)
if mode=="dsp.UDPReceiver",release(receiver);end
end
function [origin,direction,left,right]=liveGeometry()
mountAngle=[-22.5,22.5,67.5,112.5,157.5,202.5,247.5,292.5];beamAngle=[-10,10,80,100,170,190,260,280];
origin=zeros(3,10);direction=zeros(3,10);left=zeros(3,8);right=zeros(3,8);
for k=1:8,origin(:,k)=[.1*cosd(mountAngle(k));.1*sind(mountAngle(k));-.04];direction(:,k)=[cosd(beamAngle(k));sind(beamAngle(k));0];left(:,k)=[cosd(beamAngle(k)-12.5);sind(beamAngle(k)-12.5);0];right(:,k)=[cosd(beamAngle(k)+12.5);sind(beamAngle(k)+12.5);0];end
origin(:,9)=[0;0;.06];origin(:,10)=[0;0;-.05];direction(:,9)=[0;0;1];direction(:,10)=[0;0;-1];
end
function styleLiveAxes(ax),set(ax,'Color',[.07,.09,.13],'XColor','w','YColor','w','ZColor','w','GridColor',[.4,.5,.6]);end
function stopKey(source,event),if strcmpi(event.Key,'escape'),setappdata(source,'StopLiveSkyGraph',true);end;end
function value=valueOrZero(array),if isempty(array),value=0;else,value=array(end);end;end
