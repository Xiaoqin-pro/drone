function [report,cache,cacheStats] = VerifyDynamicStateFeasibility( ...
    baseModel,oldCache,state,activeIDs,refBudget,nRestarts)
%VERIFYDYNAMICSTATEFEASIBILITY 审计事件发生后的真实动态状态
%   状态包括当前位置、当前时间和事件后的活动客户集合。
model = baseModel;
model.startTime = state.time;
model.depot = state.position;
[cache,cacheStats] = UpdateLegCache(model,oldCache, ...
    activeIDs,state.position);
report = BuildReferenceSolution(model,cache,activeIDs, ...
    refBudget,nRestarts);
report.stateTime = state.time;
report.statePosition = state.position;
report.activeIDs = activeIDs(:)';
end
