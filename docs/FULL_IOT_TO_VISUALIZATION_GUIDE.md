# Full Guide — ESP32 + 10 VL53L0X Data to MATLAB Visualization

This guide follows the packet and sensor IDs in `Build IoT Device.pdf`.

## Final result

```text
10 VL53L0X modules
 ↓ two TCA9548A I²C multiplexers
ESP32 NodeMCU 30-pin
 ↓ JSON UDP over Wi-Fi, port 5005
MATLAB receiver/parser
 ↓
Live body-relative ToF dashboard
 ↓ optional recording/replay
Simulink Real/Virtual ToF selector
 ↓
Tracker → Graph → GAT → CBF dashboard
```

Start with visualization only. Keep propellers removed.

---

# 0. Security action before continuing

The uploaded PDF contains a Blynk authentication token. Treat it as exposed:

1. Open Blynk Console.
2. Revoke/regenerate the device authentication token.
3. Replace the old token in local firmware.
4. Do not include the new token in reports, GitHub, screenshots or shared ZIP files.

The firmware supplied here contains no Blynk credential.

---

# 1. Understand the two firmware versions

## Wokwi version in the PDF

The PDF uses ten HC-SR04 sensors as stand-ins because Wokwi does not provide the intended VL53L0X/multiplexer arrangement. It already transmits JSON:

```json
{"seq":42,"t":123456,"ms":48,"r":[500,510,-1,750,800,900,1000,1100,1200,1300],"st":[0,0,1,0,0,0,0,0,0,0]}
```

This packet is supported by the MATLAB parser in this package.

## Physical version

Use:

```text
ESP32 + 2 TCA9548A + 10 VL53L0X
```

Use `ESP32_10x_VL53L0X_UDP.ino`, not the HC-SR04 pin table from Wokwi.

---

# 2. Additional physical components required

You currently list ESP32, ten sensors, breadboard and wires. Add:

```text
2 × TCA9548A I²C multiplexer
1 × regulated 3.3 V supply with adequate current margin
local/bulk decoupling
safe USB/bench power
```

A half breadboard is acceptable for stationary bench testing, not for flight.

---

# 3. Wiring

## ESP32 I²C

For the selected common 30-pin ESP32 layout, begin with:

```text
GPIO21 -> SDA
GPIO22 -> SCL
GND -> common ground
```

Verify the printed labels on the exact board before power-up.

## Multiplexer addresses

```text
TCA #1: address 0x70, A0=A1=A2=LOW
TCA #2: address 0x71, A0=HIGH, A1=A2=LOW
```

Both multiplexers:

```text
VCC -> regulated 3.3 V
GND -> common ground
SDA -> ESP32 GPIO21
SCL -> ESP32 GPIO22
RESET -> valid pull-up to 3.3 V, according to module
```

## Sensors

Power each selected breakout from the verified 3.3 V sensor rail:

```text
VIN -> regulated 3.3 V
GND -> common ground
SDA/SCL -> its own mux channel
```

Mapping:

```text
0x70 channel 0 -> H0 FRONT_RIGHT
0x70 channel 1 -> H1 FRONT_LEFT
0x70 channel 2 -> H2 LEFT_FRONT
0x70 channel 3 -> H3 LEFT_REAR
0x70 channel 4 -> H4 REAR_LEFT
0x70 channel 5 -> H5 REAR_RIGHT
0x70 channel 6 -> H6 RIGHT_REAR
0x70 channel 7 -> H7 RIGHT_FRONT
0x71 channel 0 -> V0 UP
0x71 channel 1 -> V1 DOWN
```

The firmware disables every mux channel before selecting one sensor. This prevents two default-address sensors from appearing simultaneously.

Do not power all ten modules from an unverified ESP32 3.3 V regulator.

---

# 4. Arduino IDE setup

1. Install Arduino IDE 2.x.
2. Add/install Espressif ESP32 board support.
3. Select the board matching your NodeMCU ESP32; commonly `ESP32 Dev Module` for initial testing.
4. Install library through Library Manager:

 ```text
 VL53L0X by Pololu
 ```

5. Open `ESP32_10x_VL53L0X_UDP.ino`.
6. Do not connect flight motors or propellers.

---

# 5. Find the MATLAB computer IPv4 address

On Windows Command Prompt:

```text
ipconfig
```

Find the IPv4 address of the Wi-Fi adapter connected to the same router as ESP32, for example:

```text
192.168.1.100
```

Do not use:

```text
127.0.0.1
ESP32 IP
router gateway
host.wokwi.internal for physical ESP32
```

---

# 6. Configure physical firmware

Edit:

```cpp
const char* WIFI_SSID = "REPLACE_WITH_WIFI_NAME";
const char* WIFI_PASSWORD = "REPLACE_WITH_WIFI_PASSWORD";
IPAddress MATLAB_PC_IP(192,168,1,100);
```

Keep:

```cpp
const uint16_t MATLAB_PORT = 5005;
```

Upload firmware and open Serial Monitor at:

```text
115200 baud
```

Expected startup:

```text
ESP32 IP: ...
Sensor 0 ready...
...
Sensor 9 ready...
```

Failed sensors show `INITIALIZATION FAILED` and status 4.

---

# 7. Windows firewall

Allow MATLAB through Windows Defender Firewall for the current private network, or create an inbound UDP rule for:

```text
UDP port 5005
```

Keep ESP32 and laptop on the same Wi-Fi network. Guest networks may block device-to-device traffic.

Test with mobile hotspot/private router if normal Wi-Fi uses client isolation.

---

# 8. Install MATLAB bridge files

Put these files in one folder:

```text
parseSkyGraphJSON.m
testSkyGraphJSONParser.m
runLiveSkyGraphDashboard.m
prepareRealToFReplay.m
tofSourceSelectorRealVirtual.m
```

Add it:

```matlab
bridgeFolder = "C:\Users\ASUS\MATLAB\drone_from_scratch\skygraph_real_sensor_bridge";
addpath(bridgeFolder,"-begin");
```

Run parser test:

```matlab
testSkyGraphJSONParser
```

Expected:

```text
SkyGraph JSON parser: PASS
```

---

# 9. Check UDP support

```matlab
which udpport
which dsp.UDPReceiver
```

The supplied dashboard uses `udpport` when available. Otherwise it tries `dsp.UDPReceiver`, which is available with supported DSP System Toolbox installations.

---

# 10. Start MATLAB before ESP32 streaming

Run:

```matlab
capture = runLiveSkyGraphDashboard(5005);
```

Then reset/power the ESP32.

The dashboard shows:

- paired body-relative cone/ray map;
- ten individual ranges;
- six nearest direction histories;
- up/down ranges;
- packet sequence;
- packet age;
- measured packet rate;
- ESP32 scan time;
- valid/alive sensor count;
- dropped/duplicate packets;
- parse errors;
- sender address.

Controls:

```text
Esc or close figure -> stop and save capture
```

The capture is saved as:

```text
real_tof_capture_YYYYMMDD_HHMMSS.mat
```

---

# 11. Verify physical directions

Use one flat target at approximately 0.5 m.

Test in order:

```text
front centre -> H0 and/or H1
left centre -> H2 and/or H3
rear centre -> H4 and/or H5
right centre -> H6 and/or H7
above -> V0
below -> V1
```

Then move the target into diagonal blind sectors. The dashboard should show no valid horizontal return when the target is outside every cone.

If the wrong direction responds, correct physical mounting/channel mapping rather than renaming data in MATLAB.

---

# 12. Packet interpretation

JSON fields:

```text
seq sequence number
t ESP32 millis timestamp
ms total scan duration in milliseconds
r ten ranges in millimetres; -1 means invalid
st ten statuses
```

Status used by the physical firmware:

```text
0 valid range
1 timeout/no target
2 outside configured 30–2000 mm range
4 sensor initialization failure
```

MATLAB displays invalid range as 2.0 m but sets `valid=false`. Never use the distance without checking validity.

The current JSON prototype has no CRC. Add binary packet/CRC only after the basic visualization works.

---

# 13. Wokwi data test

The PDF's Wokwi sketch sends the same JSON format. Start the MATLAB dashboard on port 5005 and run Wokwi.

If the browser simulation cannot reach local MATLAB directly, use the appropriate Wokwi gateway/local VS Code setup, or test the physical ESP32 on the same LAN. Blynk data appearing on the phone does not prove UDP can reach MATLAB.

Do not reuse the exposed Blynk token from the PDF.

---

# 14. Record for at least five minutes

Acceptance checks:

```text
parse errors = 0
sensor channel order correct
valid mask follows target movement
packet rate measured
scan time measured
packet age normally below 0.15 s
dropped packets documented
no sensor permanently stuck
```

For an extended test, record 30 minutes.

---

# 15. Convert capture into Simulink replay

```matlab
replay = prepareRealToFReplay("real_tof_capture_20260828_120000.mat",0.05);
```

Use your actual filename.

This creates base-workspace timeseries:

```text
realDistanceTS width 10
realValidTS width 10
realStatusTS width 10
realPacketNewTS width 1
realPacketAgeTS width 1
realSequenceTS width 1
```

The conversion resamples irregular packet arrivals onto a 20 Hz timeline, holds the last frame, calculates packet age and invalidates stale frames after 0.15 s.

---

# 16. Create a Simulink replay model

Copy the latest completed model and save as:

```text
quadrotor_real_tof_replay.slx
```

Add six **From Workspace** blocks:

```text
realDistanceTS
realValidTS
realStatusTS
realPacketNewTS
realPacketAgeTS
realSequenceTS
```

Set interpolation off/zero-order hold where the block offers that option.

---

# 17. Add Real/Virtual ToF selector

Add MATLAB Function block:

```text
ToF Source Selector
```

Code:

```matlab
function [distance,valid,status,packetNew,packetAge,source] = ...
    fcn(mode,vDistance,vValid,vStatus,vNew,vAge, ...
    rDistance,rValid,rStatus,rNew,rAge)
%#codegen
[distance,valid,status,packetNew,packetAge,source] = ...
    tofSourceSelectorRealVirtual(mode,vDistance,vValid,vStatus,vNew,vAge, ...
    rDistance,rValid,rStatus,rNew,rAge);
end
```

Sizes:

```text
mode 1
distances 10
valid 10 boolean/status real valid may arrive as double and should be converted
status 10
packetNew 1
packetAge 1
source 1
```

Add Constant:

```text
ToF Source Mode = 0 virtual
ToF Source Mode = 1 real replay
```

---

# 18. Rewire Stage 9 tracker

Old:

```text
Stage 8 Realistic ToF -> Stage 9 Tracker
```

New:

```text
Stage 8 Realistic ToF ──┐
 ├─> ToF Source Selector -> Stage 9 Tracker
Real replay signals ────┘
```

Connect selected:

```text
distance -> Tracker distanceM
valid -> Tracker valid
status -> Tracker status
packetNew -> Tracker packetNew
packetAge -> Tracker packetAge
```

Log selected outputs under new names such as:

```text
selected_tof_distance_log
selected_tof_valid_log
selected_tof_source_log
```

---

# 19. Prevent virtual-source leakage in real mode

The Stage 8 Scenario Manager contains virtual telemetry/map entities. They do not exist in the physical room.

For the first real replay test, gate these tracker source inputs to zero/false when `ToF Source Mode=1`:

```text
sourcePositionVector = zeros(48,1)
sourceVelocityVector = zeros(48,1)
sourceRadius = zeros(16,1)
sourceId = zeros(16,1)
sourceActive = false(16,1)
telemetryAvailable = false(16,1)
mapAvailable = false(16,1)
```

Then every unmatched physical ToF detection becomes an unknown track. Add real cooperative telemetry/map sources later.

Do not visualize virtual scenario entity labels as real detections.

---

# 20. Fixed-pose versus world-pose visualization

## Stationary clone

Use body-relative dashboard only, or define a fixed visualization pose:

```text
position [0,0,1]
attitude [0,0,0]
```

## Moving physical drone

Stream synchronized flight-controller/motion-capture pose. Without it, world hit points and tracks are not geometrically valid.

Never combine a moving simulated pose with stationary physical sensors and describe the world points as real.

---

# 21. Live Simulink comes after replay

Recommended order:

```text
live ESP32 -> standalone MATLAB dashboard
record capture
capture -> Simulink replay
validate tracker/graph/GAT/CBF shadow mode
then add live UDP Receive subsystem
```

Replay is easier to debug and repeat. Do not begin with live packets directly controlling Simulink logic.

---

# 22. Final pass criteria

```text
JSON parser test passes
all ten sensor IDs map correctly
front/left/rear/right/up/down physical tests pass
normal packet age < 0.15 s
stale packets invalidate all real valid flags
capture saves and reloads
20 Hz replay timeseries are created
source selector switches without dimension errors
real mode does not receive virtual IDs/types/map entities
Stage 9 creates unknown tracks from real replay
GAT/CBF run in visualization/shadow mode only
no physical motor command depends on this link
```
