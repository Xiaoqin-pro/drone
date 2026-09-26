function results = RunPostFeasibleNeighborhoodForkAudit
%RUNPOSTFEASIBLENEIGHBORHOODFORKAUDIT 从同一first-feasible路线比较后续邻域族

root=fileparts(fileparts(mfilename('fullpath'))); setup;
benchmarkDir=fullfile(root,'data','tsp_tw_benchmark');
manifest=readtable(fullfile(benchmarkDir,'manifest.csv'),'Delimiter',',','TextType','string');
methods=["GlobalRelocate","Swap","2opt","VND"];
seeds=1:10; rows=cell(0,1);
for i=1:height(manifest)
    instance=ReadTSPTWInstance(fullfile(benchmarkDir,manifest.file(i)));
    firstBudget=50*instance.nCustomers; forkBudget=5*instance.nCustomers;
    for seedIndex=1:numel(seeds)
        seed=995000+100*i+seedIndex; rng(seed,'twister');
        runOptions=struct('nPop',20,'maxFE',firstBudget, ...
            'localSearchMode','state-switch','localSearchFE',max(1,instance.nCustomers-1), ...
            'latePenalty',1000,'silent',true);
        [~,~,stats]=RandomKeyTSPTWPSO(instance,runOptions,[]);
        if isempty(stats.firstFeasibleRoute), continue; end
        route=stats.firstFeasibleRoute; detail=stats.firstFeasibleDetail;
        for m=1:numel(methods)
            switch methods(m)
                case "GlobalRelocate", mode='global';
                case "Swap", mode='swap';
                case "2opt", mode='2opt';
                otherwise, mode='mixed';
            end
            searchOptions=struct('mode',mode,'maxFE',forkBudget,'latePenalty',1000);
            [~,afterDetail,localStats]=DiscreteRouteSearch(route,instance, ...
                searchOptions,detail);
            cost=afterDetail.tourCost;
            gain=max(0,detail.tourCost-cost);
            rows{end+1,1}={manifest.name(i),instance.nCustomers,seed, ...
                stats.firstFeasibleFE,methods(m),forkBudget,detail.tourCost,cost, ...
                gain,gain/max(localStats.functionEvaluations,1),afterDetail.isFeasible, ...
                localStats.functionEvaluations}; %#ok<AGROW>
        end
    end
    fprintf('%s completed.\n',manifest.name(i));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'instance','nCustomers','seed','first_feasible_fe','method','fork_fe', ...
    'first_feasible_cost','final_cost','cost_gain','gain_per_fe','is_feasible', ...
    'local_search_fe'});
results.summary=summary; results.methods=methods;
writetable(summary,fullfile(root,'results','post_feasible_neighborhood_fork_summary.csv'));
save(fullfile(root,'results','post_feasible_neighborhood_fork_result.mat'),'results');
fprintf('Post-feasible neighborhood fork completed: %d rows.\n',height(summary));
end

