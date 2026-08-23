function summary=plotStage15Results(results)
%PLOTSTAGE15RESULTS Create comparison plots and aggregated table.

valid=results.RunSucceeded;
if ~any(valid),error('No Stage 15 simulation completed successfully.');end
r=results(valid,:);
labels=categorical(r.Name);

figure('Name','Stage 15 mission and safety');
subplot(2,1,1);bar(labels,double(r.MissionSuccess));ylim([0,1.1]);
ylabel('Mission success');grid on;xtickangle(35);
subplot(2,1,2);bar(labels,r.MinimumSeparation);hold on;yline(0,'--r');
ylabel('Minimum separation (m)');grid on;xtickangle(35);

figure('Name','Stage 15 true versus tracked separation');
bar(labels,[r.MinimumSeparation,r.TrackedMinimumSeparation]);hold on;yline(0,'--r');
ylabel('Minimum separation (m)');grid on;xtickangle(35);
legend('Ground-truth entity clearance','Tracker/CBF clearance','Location','best');

figure('Name','Stage 15 path and control cost');
subplot(2,1,1);bar(labels,r.PathLength);ylabel('Path length (m)');grid on;xtickangle(35);
subplot(2,1,2);bar(labels,r.ControlEffort);ylabel('Control effort');grid on;xtickangle(35);

figure('Name','Stage 15 CBF behavior');
subplot(3,1,1);bar(labels,100*r.CBFInterventionRate);ylabel('Intervention (%)');grid on;xtickangle(35);
subplot(3,1,2);bar(labels,r.MaximumSlack);ylabel('Maximum slack');grid on;xtickangle(35);
subplot(3,1,3);bar(labels,r.FallbackCount);ylabel('Fallback samples');grid on;xtickangle(35);

% Aggregate by policy/safety mode.
summary = table();
for policy = 0:1
    for safety = 0:1
        selection = r.PolicyMode==policy & r.SafetyMode==safety;
        if ~any(selection), continue; end

        newRow = table( ...
            policy, ...
            safety, ...
            sum(selection), ...
            mean(r.MissionSuccess(selection)), ...
            mean(r.MinimumSeparation(selection) < 0), ...
            mean(r.MinimumSeparation(selection)), ...
            mean(r.PathLength(selection)), ...
            mean(r.ControlEffort(selection)), ...
            mean(r.CBFInterventionRate(selection)), ...
            mean(r.StalePacketFraction(selection)), ...
            'VariableNames', { ...
                'PolicyMode','SafetyMode','RunCount','SuccessRate', ...
                'CollisionRate','MeanMinimumSeparation','MeanPathLength', ...
                'MeanControlEffort','MeanInterventionRate','MeanStaleFraction'});

        summary = [summary; newRow]; %#ok<AGROW>
    end
end
% The summary table is written to the shared <projectRoot>/data folder.
thisFolder=fileparts(mfilename('fullpath'));
outputFolder=fullfile(fileparts(thisFolder),'data');
writetable(summary,fullfile(outputFolder,'stage15_summary.csv'));

fprintf('\nSTAGE 15 AGGREGATED SUMMARY\n');
disp(summary);
fprintf('PolicyMode: 0 waypoint, 1 PPO\n');
fprintf('SafetyMode: 0 bypass, 1 CBF applied\n');
end
