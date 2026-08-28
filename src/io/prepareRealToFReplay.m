function replay=prepareRealToFReplay(captureFile,sampleTime)
%PREPAREREALTOFREPLAY Convert irregular capture into 20 Hz held timeseries.

if nargin<2,sampleTime=0.05;end
loaded=load(captureFile,'capture');capture=loaded.capture;
if isempty(capture.time),error('Capture contains no packets.');end
uniformTime=(0:sampleTime:capture.time(end)).';N=numel(uniformTime);
distance=2*ones(N,10);valid=false(N,10);status=2*ones(N,10);
packetNew=false(N,1);packetAge=zeros(N,1);sequence=zeros(N,1);
lastIndex=0;
for k=1:N
    indices=find(capture.time<=uniformTime(k));
    if isempty(indices)
        packetAge(k)=0;
    else
        index=indices(end);
        distance(k,:)=capture.distanceM(index,:);
        valid(k,:)=capture.valid(index,:);
        status(k,:)=capture.status(index,:);
        sequence(k)=capture.sequence(index);
        packetAge(k)=uniformTime(k)-capture.time(index);
        packetNew(k)=index~=lastIndex;
        lastIndex=index;
        if packetAge(k)>0.15,valid(k,:)=false;end
    end
end
replay=struct('time',uniformTime,'distanceM',distance,'valid',valid, ...
    'status',status,'packetNew',packetNew,'packetAge',packetAge,'sequence',sequence);
realDistanceTS=timeseries(distance,uniformTime);
realValidTS=timeseries(double(valid),uniformTime);
realStatusTS=timeseries(status,uniformTime);
realPacketNewTS=timeseries(double(packetNew),uniformTime);
realPacketAgeTS=timeseries(packetAge,uniformTime);
realSequenceTS=timeseries(sequence,uniformTime);
assignin('base','realDistanceTS',realDistanceTS);assignin('base','realValidTS',realValidTS);
assignin('base','realStatusTS',realStatusTS);assignin('base','realPacketNewTS',realPacketNewTS);
assignin('base','realPacketAgeTS',realPacketAgeTS);assignin('base','realSequenceTS',realSequenceTS);
fprintf('Created real ToF replay timeseries: %.2f s, %d samples.\n',uniformTime(end),N);
end
