%% TESTSKYGRAPHJSONPARSER Validate the documented ESP32 JSON packet
payload='{"seq":42,"t":123456,"ms":48,"r":[500,510,-1,750,800,900,1000,1100,1200,1300],"st":[0,0,1,0,0,0,0,0,0,0]}';
[frame,ok,message]=parseSkyGraphJSON(payload);
assert(ok,message);
assert(frame.sequence==42 && frame.deviceTimeMs==123456 && frame.scanTimeMs==48);
assert(isequal(size(frame.distanceM),[10,1]));
assert(frame.valid(1) && abs(frame.distanceM(1)-0.5)<1e-12);
assert(~frame.valid(3) && frame.distanceM(3)==2.0);
assert(bitget(frame.validMask,1)==1 && bitget(frame.validMask,3)==0);
[~,badOk]=parseSkyGraphJSON('{"seq":1}');assert(~badOk);
fprintf('SkyGraph JSON parser: PASS\n');
