%% PLOTSTAGE9TRACKING Plot fixed-capacity entity tracker outputs
if ~exist('out','var')
    error('Run first: out = sim("quadrotor_stage9_tracking");');
end

[t,positionVector] = readStage9Log(out,'track9_position_log',48);
[~,type] = readStage9Log(out,'track9_type_log',16);
[~,active] = readStage9Log(out,'track9_active_log',16);
[~,confidence] = readStage9Log(out,'track9_confidence_log',16);
[~,age] = readStage9Log(out,'track9_age_log',16);
[~,sourceMask] = readStage9Log(out,'track9_source_log',16);
[~,closingSpeed] = readStage9Log(out,'track9_closing_log',16);
[~,ttc] = readStage9Log(out,'track9_ttc_log',16);

active = active > 0.5;

figure('Name','Stage 9 tracked entity paths');
hold on;
grid on;
axis equal;
colors = [0.15,0.68,1.0;0.58,0.42,0.25;1.0,0.30,0.20];
for track = 1:16
    trajectory = nan(length(t),3);
    lastType = 0;
    for k = 1:length(t)
        positions = reshape(positionVector(k,:).',3,16);
        if active(k,track)
            trajectory(k,:) = positions(:,track).';
            lastType = round(type(k,track));
        end
    end
    if lastType == 2
        firstValid = find(all(isfinite(trajectory),2),1,'first');
        if ~isempty(firstValid)
            plot3(trajectory(firstValid,1),trajectory(firstValid,2), ...
                trajectory(firstValid,3),'s','MarkerSize',9, ...
                'MarkerFaceColor',colors(2,:),'MarkerEdgeColor',[0.9,0.8,0.6]);
        end
    elseif lastType == 1 || lastType == 3
        plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3), ...
            'Color',colors(lastType,:),'LineWidth',1.2);
    end
end
xlabel('World X (m)');
ylabel('World Y (m)');
zlabel('World Z (m)');
title('Tracked paths: blue cooperative, brown building, red unknown');

figure('Name','Stage 9 track counts and confidence');
subplot(2,1,1);
plot(t,[sum(active & type==1,2),sum(active & type==2,2), ...
    sum(active & type==3,2)],'LineWidth',1.3);
grid on;
ylabel('Active tracks');
legend('Cooperative','Building','Unknown','Location','best');
subplot(2,1,2);
meanConfidence = sum(confidence.*active,2)./max(sum(active,2),1);
plot(t,meanConfidence,'LineWidth',1.3);
hold on;
plot(t,max(age.*active,[],2),'--','LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('Value');
legend('Mean confidence','Maximum age (s)','Location','best');

figure('Name','Stage 9 source masks');
imagesc(t,1:16,sourceMask.');
axis xy;
xlabel('Time (s)');
ylabel('Track slot');
colorbar;
title('Source mask: 1 telemetry, 2 map, 4 ToF, 5 telemetry+ToF, 6 map+ToF');

figure('Name','Stage 9 threat kinematics');
minimumTTC = 20*ones(length(t),1);
maximumClosing = zeros(length(t),1);
for k = 1:length(t)
    current = active(k,:);
    if any(current)
        minimumTTC(k) = min([ttc(k,current),20]);
        maximumClosing(k) = max(closingSpeed(k,current));
    end
end
subplot(2,1,1);
plot(t,minimumTTC,'LineWidth',1.3);
grid on;
ylabel('Minimum TTC (s)');
subplot(2,1,2);
plot(t,maximumClosing,'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Max closing speed (m/s)');

function [time,data] = readStage9Log(simulationOutput,name,width)
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
