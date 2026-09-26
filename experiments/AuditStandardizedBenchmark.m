function AuditStandardizedBenchmark
%AUDITSTANDARDIZEDBENCHMARK 检查Solomon转换为单UAV动态实例的合法性
scriptDir=fileparts(mfilename('fullpath')); root=fileparts(scriptDir);
dataDir=fullfile(root,'data','standardized_dynamic'); files=dir(fullfile(dataDir,'*_dynamic.mat')); rows=cell(0,1);
for f=1:numel(files)
    loaded=load(fullfile(dataDir,files(f).name)); instance=loaded.instance; model=instance.model;
    model.cfg.silent=true; allIDs=model.customerIDs(:)'; activeIDs=instance.activeIDs;
    cache=BuildLegCache(model,activeIDs,model.homePosition);
    % 事件前路线只包含可见客户。
    preRoute=instance.preEventRoute(:)'; preActive=preRoute(ismember(preRoute,activeIDs));
    [preCost,preDetail]=EvaluateSchedule(preActive,model,cache);
    preFeasible=preDetail.isFeasible;
    % 用事件前路线模拟真实执行状态。
    solution.Cost=preCost; solution.Detail=preDetail; solution.Route=preRoute;
    state=ExecuteUntilEvent(solution,model,instance.event.time);
    revealValid=true;
    for id=instance.hiddenIDs(:)'
        revealValid=revealValid && model.windows(id,2)>instance.event.time;
    end
    % 事件后把隐藏客户加入活动集合，使用原始可行路线的剩余部分作为witness。
    postIDs=allIDs;
    postModel=model; postModel.startTime=instance.event.time; postModel.depot=state.position;
    postCache=BuildLegCache(postModel,postIDs,state.position);
    completed=state.completedIDs(:)'; postRoute=preRoute(~ismember(preRoute,completed));
    [postCost,postDetail]=EvaluateSchedule(postRoute,postModel,postCache);
    postFeasible=postDetail.isFeasible;
    rows{end+1,1}={string(instance.sourceBenchmark),instance.customerCount, ...
        numel(activeIDs),numel(instance.hiddenIDs),instance.event.time, ...
        preFeasible,revealValid,postFeasible,preCost,postCost, ...
        numel(preRoute),numel(postRoute)}; %#ok<AGROW>
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'source','customer_count','active_count','hidden_count','event_time', ...
    'pre_event_feasible','reveal_before_due','post_event_witness_feasible', ...
    'pre_event_cost','post_event_cost','pre_route_length','post_route_length'});
writetable(summary,fullfile(root,'results','standardized_benchmark_audit.csv'));
disp(summary);
end
