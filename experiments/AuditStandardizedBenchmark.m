function AuditStandardizedBenchmark
%AUDITSTANDARDIZEDBENCHMARK 检查标准来源单UAV动态实例
scriptDir=fileparts(mfilename('fullpath')); root=fileparts(scriptDir);
dataDir=fullfile(root,'data','standardized_dynamic'); files=dir(fullfile(dataDir,'*_dynamic.mat')); rows=cell(0,1);
for f=1:numel(files)
    loaded=load(fullfile(dataDir,files(f).name)); instance=loaded.instance; model=instance.model;
    model.cfg.silent=true;
    preRoute=instance.preEventRoute(:)';
    preModel=model; preModel.startTime=0; preModel.depot=model.homePosition;
    preIDs=unique(preRoute,'stable');
    preCache=BuildLegCache(preModel,preIDs,model.homePosition);
    [preCost,preDetail]=EvaluateSchedule(preRoute,preModel,preCache);
    state=instance.preEventState;
    postModel=model; postModel.startTime=instance.event.time; postModel.depot=state.position;
    postIDs=unique([instance.activeIDs(:)',instance.hiddenIDs(:)'],'stable');
    postCache=BuildLegCache(postModel,postIDs,state.position);
    postRoute=instance.postWitnessRoute(:)';
    [postCost,postDetail]=EvaluateSchedule(postRoute,postModel,postCache);
    hiddenIncluded=all(ismember(instance.hiddenIDs(:)',postRoute));
    revealValid=all(model.windows(instance.hiddenIDs,2)>instance.event.time);
    rows{end+1,1}={string(instance.sourceBenchmark),instance.customerCount, ...
        numel(instance.activeIDs),numel(instance.hiddenIDs),instance.event.time, ...
        preDetail.isFeasible,revealValid,hiddenIncluded,postDetail.isFeasible, ...
        preCost,postCost,numel(preRoute),numel(postRoute)}; %#ok<AGROW>
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'source','customer_count','active_count','hidden_count','event_time', ...
    'pre_event_feasible','reveal_before_due','hidden_in_post_route', ...
    'post_event_witness_feasible','pre_event_cost','post_event_cost', ...
    'pre_route_length','post_route_length'});
writetable(summary,fullfile(root,'results','standardized_benchmark_audit.csv'));
disp(summary);
end
