function results = RunStateSwitchForkAudit
%RUNSTATESWITCHFORKAUDIT 从同一条first-feasible路线比较Propagation与Global后续强化

root=fileparts(fileparts(mfilename('fullpath'))); setup;
benchmarkDir=fullfile(root,'data','tsp_tw_benchmark');
manifest=readtable(fullfile(benchmarkDir,'manifest.csv'),'Delimiter',',','TextType','string');
seeds=1:10; rows=cell(0,1);
for i=1:height(manifest)
    instance=ReadTSPTWInstance(fullfile(benchmarkDir,manifest.file(i)));
    firstBudget=50*instance.nCustomers; extraBudget=5*instance.nCustomers;
    for seedIndex=1:numel(seeds)
        seed=970000+100*i+seeds(seedIndex); rng(seed,'twister');
        options=struct('nPop',20,'maxFE',firstBudget,'latePenalty',1000, ...
            'localSearchMode','propagation','localSearchFE',instance.nCustomers,'silent',true);
        [~,~,stats]=RandomKeyTSPTWPSO(instance,options,[]);
        if isempty(stats.firstFeasibleRoute), continue; end
        firstRoute=stats.firstFeasibleRoute; firstDetail=stats.firstFeasibleDetail;
        for branch=["AlwaysPropagation","StateSwitchToGlobal"]
            if branch=="AlwaysPropagation", mode='propagation'; else, mode='global'; end
            searchOptions=struct('mode',mode,'maxFE',extraBudget,'latePenalty',1000);
            [route,detail,localStats]=DiscreteRouteSearch(firstRoute,instance, ...
                searchOptions,firstDetail); %#ok<ASGLU>
            gap=(detail.tourCost-firstDetail.tourCost)/firstDetail.tourCost;
            rows{end+1,1}={manifest.name(i),instance.nCustomers,seed, ...
                stats.firstFeasibleFE,branch,firstDetail.tourCost,detail.tourCost, ...
                gap,detail.isFeasible,localStats.functionEvaluations}; %#ok<AGROW>
        end
    end
    fprintf('%s completed.\n',manifest.name(i));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'instance','nCustomers','seed','first_feasible_fe','branch', ...
    'first_feasible_cost','final_cost','post_feasible_gap','is_feasible', ...
    'local_search_fe'});
results.summary=summary;
writetable(summary,fullfile(root,'results','state_switch_fork_audit_summary.csv'));
save(fullfile(root,'results','state_switch_fork_audit_result.mat'),'results');
fprintf('State-switch fork audit completed: %d rows.\n',height(summary));
end
