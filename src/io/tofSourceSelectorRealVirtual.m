function [distance,valid,status,packetNew,packetAge,source] = ...
    tofSourceSelectorRealVirtual(mode,vDistance,vValid,vStatus,vNew,vAge, ...
    rDistance,rValid,rStatus,rNew,rAge)
%TOFSOURCESELECTORREALVIRTUAL Select identical virtual or real interface.

%#codegen

distance=zeros(10,1);valid=false(10,1);status=zeros(10,1);
packetNew=false;packetAge=0;source=0;
if mode(1)>=0.5
    distance(:)=rDistance(:);valid(:)=rValid(:);status(:)=rStatus(:);
    packetNew=rNew(1)>0.5;packetAge=rAge(1);source=1;
else
    distance(:)=vDistance(:);valid(:)=vValid(:);status(:)=vStatus(:);
    packetNew=vNew(1)>0.5;packetAge=vAge(1);source=0;
end
if packetAge>0.15
    valid(:)=false;packetNew=false;
end
end
