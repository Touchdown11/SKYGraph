function [frame,ok,message] = parseSkyGraphJSON(payload)
%PARSESKYGRAPHJSON Parse the JSON packet documented in Build IoT Device.pdf.
% Expected: {"seq":n,"t":ms,"ms":scan_ms,"r":[10],"st":[10]}

frame=struct('sequence',0,'deviceTimeMs',0,'scanTimeMs',0, ...
    'rangeMm',-ones(10,1),'distanceM',2*ones(10,1), ...
    'status',2*ones(10,1),'valid',false(10,1), ...
    'validMask',uint16(0));
ok=false;message="";
try
    data=jsondecode(char(payload));
    required={'seq','t','ms','r','st'};
    for k=1:numel(required)
        if ~isfield(data,required{k})
            message="Missing JSON field: "+required{k};return;
        end
    end
    rangeMm=double(data.r(:));status=double(data.st(:));
    if numel(rangeMm)~=10 || numel(status)~=10
        message="Fields r and st must each contain exactly 10 values";return;
    end
    if any(~isfinite(rangeMm)) || any(~isfinite(status))
        message="Range/status contains NaN or Inf";return;
    end
    frame.sequence=double(data.seq);
    frame.deviceTimeMs=double(data.t);
    frame.scanTimeMs=double(data.ms);
    frame.rangeMm=rangeMm;
    frame.status=status;
    mask=uint16(0);
    for sensor=1:10
        frame.valid(sensor)=status(sensor)==0 && rangeMm(sensor)>=30 && rangeMm(sensor)<=2000;
        if frame.valid(sensor)
            frame.distanceM(sensor)=rangeMm(sensor)/1000;
            mask=bitset(mask,sensor,1);
        else
            frame.distanceM(sensor)=2.0;
        end
    end
    frame.validMask=mask;
    ok=true;message="OK";
catch exception
    message=string(exception.message);
end
end
