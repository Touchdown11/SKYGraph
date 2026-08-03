%% PLOTSTAGE8SCENARIOS Plot scenario, source flags, ToF and fault schedule
if ~exist('out','var')
    error('Run first: out = sim("quadrotor_stage8_scenarios");');
end

[tScenario,positionVector] = readStage8Log(out,'scenario8_position_log',48);
[~,active] = readStage8Log(out,'scenario8_active_log',16);
[~,type] = readStage8Log(out,'scenario8_type_log',16);
[~,telemetry] = readStage8Log(out,'scenario8_telemetry_log',16);
[~,mapAvailable] = readStage8Log(out,'scenario8_map_log',16);
[~,phase] = readStage8Log(out,'scenario8_phase_log',1);
[~,fault] = readStage8Log(out,'scenario8_fault_log',1);
[tToF,distance] = readStage8Log(out,'tof8_distance_log',10);
[~,valid] = readStage8Log(out,'tof8_valid_log',10);
[~,packetAge] = readStage8Log(out,'tof8_packet_age_log',1);
[~,dropCount] = readStage8Log(out,'tof8_drop_count_log',1);

active = active > 0.5;
valid = valid > 0.5;
distance(~valid) = NaN;

figure('Name','Stage 8 multi-entity scene');
hold on;
grid on;
axis equal;
colors = [0.15,0.68,1.0;0.55,0.40,0.25;1.0,0.30,0.20];
for entity = 1:16
    trajectory = nan(length(tScenario),3);
    entityType = 0;
    for k = 1:length(tScenario)
        positions = reshape(positionVector(k,:).',3,16);
        if active(k,entity)
            trajectory(k,:) = positions(:,entity).';
            entityType = round(type(k,entity));
        end
    end
    if entityType == 2
        % Static building trajectories collapse to one point, so draw a
        % visible square marker instead of an invisible zero-length line.
        firstValid = find(all(isfinite(trajectory),2),1,'first');
        if ~isempty(firstValid)
            plot3(trajectory(firstValid,1),trajectory(firstValid,2), ...
                trajectory(firstValid,3),'s','MarkerSize',10, ...
                'MarkerFaceColor',colors(2,:),'MarkerEdgeColor',[0.9,0.8,0.6]);
            text(trajectory(firstValid,1),trajectory(firstValid,2), ...
                trajectory(firstValid,3)+0.12,sprintf('B%d',entity), ...
                'Color',[0.9,0.8,0.6],'HorizontalAlignment','center');
        end
    elseif entityType == 1 || entityType == 3
        plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3), ...
            'Color',colors(entityType,:),'LineWidth',1.3);
    end
end
xlabel('World X (m)');
ylabel('World Y (m)');
zlabel('World Z (m)');
title('Blue cooperative, brown mapped building, red unknown');

figure('Name','Stage 8 entity/source counts');
subplot(3,1,1);
stairs(tScenario,sum(active,2),'LineWidth',1.3);
grid on;
ylabel('Active entities');
subplot(3,1,2);
stairs(tScenario,sum(telemetry>0.5,2),'LineWidth',1.3);
hold on;
stairs(tScenario,sum(mapAvailable>0.5,2),'LineWidth',1.3);
grid on;
ylabel('Source count');
legend('Telemetry','Map','Location','best');
subplot(3,1,3);
stairs(tScenario,phase,'LineWidth',1.3);
hold on;
stairs(tScenario,fault,'--','LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('Mode');
legend('Scenario phase','Fault mode','Location','best');

figure('Name','Stage 8 realistic ToF');
plot(tToF,distance,'LineWidth',1.0);
grid on;
xlabel('Time (s)');
ylabel('Valid range (m)');
legend('H0','H1','H2','H3','H4','H5','H6','H7','Up','Down', ...
    'Location','eastoutside');

figure('Name','Stage 8 communication health');
subplot(2,1,1);
plot(tToF,packetAge,'LineWidth',1.3);
hold on;
yline(0.15,'--r','Stale threshold');
grid on;
ylabel('Packet age (s)');
subplot(2,1,2);
stairs(tToF,dropCount,'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Cumulative drops');

function [time,data] = readStage8Log(simulationOutput,name,width)
names = simulationOutput.who;
if any(strcmp(names,name))
    signal = simulationOutput.get(name);
elseif evalin('base',['exist(''' name ''',''var'')'])
    signal = evalin('base',name);
else
    error('Missing log "%s".',name);
end
time = signal.Time(:);
data = squeeze(signal.Data);
if isvector(data)
    data = data(:);
elseif size(data,1) == numel(time)
    % Correct orientation.
elseif size(data,2) == numel(time)
    data = data.';
else
    error('%s has incompatible dimensions.',name);
end
if size(data,2) < width
    error('%s needs %d columns but has %d.',name,width,size(data,2));
end
data = data(:,1:width);
end
