function results = RunFeasibleSourceOracleAudit
%RUNFEASIBLESOURCEORACLEAUDIT Oracle Audit V2
%   收集first-feasible/final-feasible路线，完整评价所有source neighborhood，
%   并计算actionable、gain、capture和random expected baseline。

root=fileparts(fileparts(mfilename('fullpath'))); setup;
benchmarkDir=fullfile(root,'data','tsp_tw_benchmark');
manifest=readtable(fullfile(benchmarkDir,'manifest.csv'),'Delimiter',',','TextType','string');
reference=ReadTSPTWReferenceSolutions(fullfile(root,'data','tsp_tw_raw','best_known', ...
    'SolomonPotvinBengio-best-known-traveltime.txt'));
routeRows=cell(0,1); sourceRows=cell(0,1); options=struct('latePenalty',1000);
for i=1:height(manifest)
    instance=ReadTSPTWInstance(fullfile(benchmarkDir,manifest.file(i)));
    refIndex=find(strcmpi(string({reference.file}),manifest.file(i)),1);
    referenceRoute=reference(refIndex).permutation+1;
    candidateRoutes={referenceRoute}; candidateLabels={"reference"}; candidateSeeds=0;
    for seedIndex=1:20
        rng(990000+100*i+seedIndex,'twister');
        runOptions=struct('nPop',20,'maxFE',100*instance.nCustomers, ...
            'localSearchMode','state-switch','localSearchFE',max(1,instance.nCustomers-1), ...
            'latePenalty',1000,'silent',true);
        [best,~,stats]=RandomKeyTSPTWPSO(instance,runOptions,[]);
        if ~isempty(stats.firstFeasibleRoute)
            candidateRoutes{end+1}=stats.firstFeasibleRoute; %#ok<AGROW>
            candidateLabels{end+1}="first-feasible"; %#ok<AGROW>
            candidateSeeds(end+1)=seedIndex; %#ok<AGROW>
        end
        if best.Detail.isFeasible
            candidateRoutes{end+1}=best.Route; %#ok<AGROW>
            candidateLabels{end+1}="final-feasible"; %#ok<AGROW>
            candidateSeeds(end+1)=seedIndex; %#ok<AGROW>
        end
    end
    routeKeys=cellfun(@(r)sprintf('%d_',r),candidateRoutes,'UniformOutput',false);
    [~,uniqueIndex]=unique(routeKeys,'stable');
    candidateRoutes=candidateRoutes(uniqueIndex); candidateLabels=candidateLabels(uniqueIndex); candidateSeeds=candidateSeeds(uniqueIndex);

    for routeIndex=1:numel(candidateRoutes)
        route=candidateRoutes{routeIndex};
        [routeCost,routeDetail]=EvaluateTSPTWRoute(route,instance,options);
        n=numel(route); sourceCosts=zeros(1,n);
        sourceDetails=cell(1,n);
        for sourceIndex=1:n
            [~,sourceCost,sourceDetail,~]=EvaluateSingleSourceRelocate( ...
                route,instance,route(sourceIndex),routeDetail,options);
            sourceCosts(sourceIndex)=sourceCost; sourceDetails{sourceIndex}=sourceDetail;
        end
        gains=routeCost-sourceCosts;
        actionable=any(gains>1e-10);
        [oracleCost,oracleIndex]=min(sourceCosts); oracleGain=max(gains);
        oracleSource=route(oracleIndex);
        randomExpectedGain=mean(max(gains,0));
        randomImproveProbability=mean(gains>1e-10);
        if actionable
            captureText="actionable";
        else
            captureText="non-actionable";
        end
        routeRows{end+1,1}={manifest.name(i),routeIndex,string(candidateLabels{routeIndex}), ...
            candidateSeeds(routeIndex),routeCost,oracleSource,oracleCost,oracleGain, ...
            randomExpectedGain,randomImproveProbability,actionable,captureText,n}; %#ok<AGROW>
        removalSaving=RemovalSavingScores(route,instance);
        for sourceIndex=1:n
            k=sourceIndex; rec=routeDetail.records(k,:);
            slack=instance.windows(route(k),2)-rec(3);
            sourceRows{end+1,1}={manifest.name(i),routeIndex,string(candidateLabels{routeIndex}), ...
                candidateSeeds(routeIndex),route(k),sourceCosts(k),max(0,gains(k)), ...
                max(0,removalSaving(k)),slack,max(0,rec(4)),max(0,rec(5)), ...
                sourceIndex==oracleIndex,actionable}; %#ok<AGROW>
        end
    end
    fprintf('%s: collected %d feasible routes.\n',manifest.name(i),numel(candidateRoutes));
end
routeSummary=cell2table(vertcat(routeRows{:}),'VariableNames',{ ...
    'instance','route_index','route_source','seed','route_cost','oracle_source', ...
    'oracle_cost','oracle_gain','random_expected_gain','random_improve_probability', ...
    'actionable','actionability_label','nCustomers'});
sourceSummary=cell2table(vertcat(sourceRows{:}),'VariableNames',{ ...
    'instance','route_index','route_source','seed','source_node','source_cost', ...
    'source_gain','removal_saving','slack','late','waiting','is_oracle_source', ...
    'actionable'});
results.routeSummary=routeSummary; results.sourceSummary=sourceSummary;
writetable(routeSummary,fullfile(root,'results','feasible_source_oracle_v2_routes.csv'));
writetable(sourceSummary,fullfile(root,'results','feasible_source_oracle_v2_sources.csv'));
save(fullfile(root,'results','feasible_source_oracle_v2_result.mat'),'results','-v7.3');
fprintf('Feasible source oracle V2 completed: %d routes, %d source rows.\n', ...
    height(routeSummary),height(sourceSummary));
end

function scores=RemovalSavingScores(route,instance)
route=route(:)'; n=numel(route); scores=zeros(1,n);
for k=1:n
    prev=instance.depotIndex; next=instance.depotIndex;
    if k>1, prev=route(k-1); end
    if k<n, next=route(k+1); end
    scores(k)=instance.costMatrix(prev,route(k))+instance.costMatrix(route(k),next) ...
        -instance.costMatrix(prev,next);
end
end
