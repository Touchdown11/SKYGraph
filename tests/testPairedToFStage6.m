%% TESTPAIREDTOFSTAGE6 Validate the clean paired-sensor arrangement
clear;
clc;
close all;

xTrue = zeros(16,1);
xTrue(3) = 1.0;
[distance,valid,status,directionDistance,directionValid,nearestSensor, ...
    entityId,entityType] = pairedToFArrayStage6(xTrue,0.0);

assert(isequal(size(distance),[10,1]));
assert(isequal(size(valid),[10,1]));
assert(isequal(size(directionDistance),[6,1]));
assert(all(distance >= 0.03 & distance <= 2.0));
assert(all(valid));
assert(all(directionValid));
assert(all(entityId(1:2) == 201));
assert(all(entityId(3:4) == 101));
assert(all(entityId(5:6) == 301));
assert(all(entityId(7:8) == 202));
assert(entityId(9) == 302);
assert(entityId(10) == 9001);

fprintf('\nCLEAN PAIRED TOF VALIDATION\n');
fprintf('===========================\n');
fprintf('Fixed dimensions: PASS\n');
fprintf('Front pair detects front object: PASS\n');
fprintf('Left pair detects left object: PASS\n');
fprintf('Rear pair detects rear object: PASS\n');
fprintf('Right pair detects right object: PASS\n');
fprintf('Up/down detection: PASS\n\n');

sensorNames = {'H0 FRONT RIGHT','H1 FRONT LEFT','H2 LEFT FRONT', ...
    'H3 LEFT REAR','H4 REAR LEFT','H5 REAR RIGHT', ...
    'H6 RIGHT REAR','H7 RIGHT FRONT','V0 UP','V1 DOWN'};
fprintf('ID  Name             Valid  Range  HitID  Type  Status\n');
for sensor = 1:10
    fprintf('%2d  %-16s %5d  %5.3f  %5.0f  %4.0f  %6.0f\n', ...
        sensor-1,sensorNames{sensor},valid(sensor),distance(sensor), ...
        entityId(sensor),entityType(sensor),status(sensor));
end

fprintf('\nCoarse direction ranges [front left rear right up down]:\n');
disp(directionDistance.');
fprintf('Nearest zero-based sensor IDs:\n');
disp(nearestSensor.');
fprintf('RESULT: PASS - paired geometry is ready for Simulink.\n\n');

% Top-view mounting and optical cone plot.
mountAzimuth = [-22.5,22.5,67.5,112.5,157.5,202.5,247.5,292.5];
beamAzimuth = [-10,10,80,100,170,190,260,280];
mountRadius = 0.10;
halfFov = 12.5;
maximumRange = 2.0;
colors = lines(8);

figure('Name','Clean paired ToF layout');
hold on;
axis equal;
grid on;
for sensor = 1:8
    origin = mountRadius*[cosd(mountAzimuth(sensor)),sind(mountAzimuth(sensor))];
    edges = beamAzimuth(sensor)+[-halfFov,halfFov];
    x = [origin(1),origin(1)+maximumRange*cosd(edges(1)), ...
        origin(1)+maximumRange*cosd(edges(2)),origin(1)];
    y = [origin(2),origin(2)+maximumRange*sind(edges(1)), ...
        origin(2)+maximumRange*sind(edges(2)),origin(2)];
    patch(x,y,colors(sensor,:),'FaceAlpha',0.12, ...
        'EdgeColor',colors(sensor,:));
    plot(origin(1),origin(2),'ko','MarkerFaceColor','k');
    text(origin(1)*1.7,origin(2)*1.7,sprintf('H%d',sensor-1), ...
        'HorizontalAlignment','center','FontWeight','bold');
end
rectangle('Position',[-0.11,-0.085,0.22,0.17], ...
    'Curvature',0.2,'LineWidth',2);
plot([1.35,0,-1.25,0],[0,1.30,0,-1.40],'rx', ...
    'MarkerSize',10,'LineWidth',2);
xlabel('Body X forward (m)');
ylabel('Body Y left (m)');
title('Mounts are 45° apart within each pair; beams are 20° apart');
xlim([-2.2,2.2]);
ylim([-2.2,2.2]);

% Demonstrate that large diagonal gaps remain.
angles = (0:0.5:359.5).';
covered = false(size(angles));
for sensor = 1:8
    difference = atan2d(sind(angles-beamAzimuth(sensor)), ...
        cosd(angles-beamAzimuth(sensor)));
    covered = covered | abs(difference) <= halfFov;
end
figure('Name','Paired layout angular coverage');
stairs(angles,double(covered),'LineWidth',1.5);
grid on;
xlabel('Body bearing (degrees)');
ylabel('Nominal coverage');
yticks([0,1]);
yticklabels({'Blind','Covered'});
title('Cardinal directions are covered; diagonal blind sectors remain');
