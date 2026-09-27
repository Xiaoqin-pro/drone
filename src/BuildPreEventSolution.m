function [solution,model,cache] = BuildPreEventSolution(mode)
%BUILDPREEVENTSOLUTION 重建动态基准生成时使用的事件前旧计划
if nargin<1 || isempty(mode), mode='legacy'; end
model=CreateModel(mode);
if strcmpi(mode,'local-terrain')
    model.windows(:,2)=model.windows(:,2)+3000;
else
    model.windows(:,2)=model.windows(:,2)+300;
end
model.cfg.silent=true;
activeIDs=model.customerIDs(:)';
cache=BuildLegCache(model,activeIDs,model.homePosition);
rng(860000,'twister');
[solution,~,~]=RoutingPSO(model,cache,100,40,0.90,0.995,1.7,1.7,[]);
end
