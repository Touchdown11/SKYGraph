function figureHandle=plotMultiEgoStage16(output)
%PLOTMULTIEGOSTAGE16 Plot common-world trajectories and safety diagnostics.
N=size(output.Position,2); figureHandle=figure('Name','SKYGraph Multi-Ego Stage 16','Color','w');
tiledlayout(1,2,'Padding','compact'); nexttile; hold on; grid on; view(3); axis equal;
colors=lines(N);
for ego=1:N
    p=squeeze(output.Position(:,ego,:)); plot3(p(1,:),p(2,:),p(3,:),'Color',colors(ego,:),'LineWidth',1.8);
    plot3(p(1,1),p(2,1),p(3,1),'o','Color',colors(ego,:),'MarkerFaceColor',colors(ego,:));
    g=output.Goal(:,ego); plot3(g(1),g(2),g(3),'x','Color',colors(ego,:),'LineWidth',2,'MarkerSize',9);
end
xlabel('X (m)');ylabel('Y (m)');zlabel('Z (m)');title('All ego trajectories (circle = start, x = goal)');
nexttile; hold on;grid on; plot(output.Time,output.MinimumPairwiseSeparation,'k','LineWidth',1.5);
yline(0,'r--','Collision boundary'); yline(.18,'Color',[.85 .5 0],'LineStyle','--','Label','margin');
xlabel('Time (s)');ylabel('Minimum surface separation (m)');title(sprintf('Joint mission success: %d',output.MissionSuccess));
end
