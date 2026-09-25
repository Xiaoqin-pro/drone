function PlotDynamicHierarchical(model,solutions,states,events,cfg)
%PLOTDYNAMICHIERARCHICAL 绘制动态事件前后的分层路线
f = figure('Visible','off','Color','w');
surf(model.X,model.Y,model.terrainZ,'EdgeColor','none', ...
    'FaceAlpha',0.73);
hold on;
shading interp;
colormap(turbo(256));
colorbar;
camlight headlight;
lighting gouraud;

for k = 1:model.nObstacles
    DrawObstacle(model.obstacles(k));
end

colors = [0.35 0.35 0.35; 0.00 0.25 0.85; 0.10 0.60 0.20];
styles = {'--','-','-.'};
for k = 1:numel(solutions)
    if isempty(solutions{k})
        continue;
    end
    points = solutions{k}.Detail.pathPoints;
    plot3(points(:,1),points(:,2),points(:,3), ...
        styles{min(k,numel(styles))},'Color',colors(min(k,size(colors,1)),:), ...
        'LineWidth',2.2);
end

for k = 1:numel(states)
    state = states{k};
    plot3(state.position(1),state.position(2),state.position(3), ...
        'p','MarkerSize',12,'MarkerFaceColor',[0.90 0.10 0.10], ...
        'MarkerEdgeColor','k');
end

for c = 1:model.nCustomers
    xyz = model.customerXYZ(c,:);
    plot3(xyz(1),xyz(2),xyz(3),'ko','MarkerFaceColor','w','MarkerSize',4);
    text(xyz(1)+8,xyz(2)+8,xyz(3)+8,sprintf('C%d',c));
end

xlabel('X');
ylabel('Y');
zlabel('Elevation Z');
title('Hierarchical dynamic routing-path replanning');
grid on;
axis tight;
daspect([1 1 1.15]);
view(-38,45);
set(gca,'Projection','perspective');
legend('Terrain','Obstacle','Initial plan','After event 1','After event 2', ...
    'Event state 1','Event state 2','Location','best');
exportgraphics(f,fullfile(cfg.outputDir, ...
    'hierarchical_dynamic_route.png'),'Resolution',180);
end

function DrawObstacle(obs)
vertices = [obs.xMin obs.yMin obs.zMin;obs.xMax obs.yMin obs.zMin; ...
            obs.xMax obs.yMax obs.zMin;obs.xMin obs.yMax obs.zMin; ...
            obs.xMin obs.yMin obs.zMax;obs.xMax obs.yMin obs.zMax; ...
            obs.xMax obs.yMax obs.zMax;obs.xMin obs.yMax obs.zMax];
faces = [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
patch('Vertices',vertices,'Faces',faces,'FaceColor',[0.85 0.25 0.12], ...
    'FaceAlpha',0.42,'EdgeColor',[0.45 0.05 0.02],'LineWidth',0.8);
end
