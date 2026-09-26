function [solution,model,cache] = BuildPreEventSolution
%BUILDPREEVENTSOLUTION 重建动态基准生成时使用的事件前旧计划
model=CreateModel();
model.windows(:,2)=model.windows(:,2)+300;
model.cfg.silent=true;
activeIDs=model.customerIDs(:)';
cache=BuildLegCache(model,activeIDs,model.homePosition);
rng(860000,'twister');
[solution,~,~]=RoutingPSO(model,cache,100,40,0.90,0.995,1.7,1.7,[]);
end
