function results = RunFeasibleSourceOracleAudit
%RUNFEASIBLESOURCEORACLEAUDIT 评估可行路线下source选择信息的预测能力
%   该实验不改变算法，只离线完整评价每个source的reinsertion neighborhood。

root=fileparts(fileparts(mfilename('fullpath'))); setup;
benchmarkDir=fullfile(root,'data','tsp_tw_benchmark');
manifest=readtable(fullfile(benchmarkDir,'manifest.csv'),'Delimiter',',','TextType','string');
reference=ReadTSPTWReferenceSolutions(fullfile(root,'data','tsp_tw_raw','best_known', ...
    'SolomonPotvinBengio-best-known-traveltime.txt'));
rows=cell(0,1); routeRows=cell(0,1); options=struct('latePenalty',1000);
for i=1:height(manifest)
    instance=ReadTSPTWInstance(fullfile(benchmarkDir,manifest.file(i)));
    refIndex=find(strcmpi(string({reference.file}),manifest.file(i)),1);
    referenceRoute=reference(refIndex).permutation+1;
    candidateRoutes={referenceRoute}; candidateLabels={"reference"};
    % 补充少量算法得到的可行路线，避免只审计官方路线。
    for seedIndex=1:10
        rng(990000+100*i+seedIndex,'twister');
        runOptions=struct('nPop',20,'maxFE',100*instance.nCustomers, ...
            'localSearchMode','state-switch','localSearchFE',max(1,instance.nCustomers-1), ...
            'latePenalty',1000,'silent',true);
        [best,~,~]=RandomKeyTSPTWPSO(instance,runOptions,[]);
        if best.Detail.isFeasible
            candidateRoutes{end+1}=best.Route; %#ok<AGROW>
            candidateLabels{end+1}="state-switch"; %#ok<AGROW>
        end
    end
    routeKeys=cellfun(@(r)sprintf('%d_',r),candidateRoutes,'UniformOutput',false);
    [~,uniqueIndex]=unique(routeKeys,'stable');
    candidateRoutes=candidateRoutes(uniqueIndex); candidateLabels=candidateLabels(uniqueIndex);

    for routeIndex=1:numel(candidateRoutes)
        route=candidateRoutes{routeIndex};
        [routeCost,routeDetail]=EvaluateTSPTWRoute(route,instance,options);
        n=numel(route); sourceCosts=zeros(1,n);
        for sourceIndex=1:n
            [~,sourceCost,~,~]=EvaluateSingleSourceRelocate( ...
                route,instance,route(sourceIndex),routeDetail,options);
            sourceCosts(sourceIndex)=sourceCost;
        end
        [oracleCost,oracleIndex]=min(sourceCosts);
        oracleSource=route(oracleIndex);
        removalSaving=RemovalSavingScores(route,instance);
        [~,removalIndex]=max(removalSaving); removalSource=route(removalIndex);
        rng(991000+100*i+routeIndex,'twister'); randomIndex=randi(n); randomSource=route(randomIndex);
        sourceRules={"Random",randomIndex,randomSource; ...
            "RemovalSaving",removalIndex,removalSource};
        [~,oracleOrder]=sort(sourceCosts,'ascend');
        for ruleIndex=1:size(sourceRules,1)
            rule=sourceRules{ruleIndex,1}; selectedIndex=sourceRules{ruleIndex,2};
            selectedCost=sourceCosts(selectedIndex);
            rank=find(oracleOrder==selectedIndex,1);
            regret=(selectedCost-oracleCost)/max(abs(oracleCost),eps);
            rows{end+1,1}={manifest.name(i),routeIndex,string(candidateLabels{routeIndex}), ...
                routeCost,rule,sourceRules{ruleIndex,3},oracleSource,rank, ...
                selectedCost,oracleCost,regret,selectedCost<routeCost-1e-10}; %#ok<AGROW>
        end
        routeRows{end+1,1}={manifest.name(i),routeIndex,string(candidateLabels{routeIndex}), ...
            routeCost,oracleSource,oracleCost,n}; %#ok<AGROW>
    end
    fprintf('%s: audited %d feasible routes.\n',manifest.name(i),numel(candidateRoutes));
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'instance','route_index','route_source','route_cost','rule','selected_source', ...
    'oracle_source','oracle_rank','selected_cost','oracle_cost','normalized_regret', ...
    'improves_route'});
routeSummary=cell2table(vertcat(routeRows{:}),'VariableNames',{ ...
    'instance','route_index','route_source','route_cost','oracle_source', ...
    'oracle_cost','nCustomers'});
results.summary=summary; results.routeSummary=routeSummary;
writetable(summary,fullfile(root,'results','feasible_source_oracle_audit_summary.csv'));
writetable(routeSummary,fullfile(root,'results','feasible_source_oracle_routes.csv'));
save(fullfile(root,'results','feasible_source_oracle_audit_result.mat'),'results');
fprintf('Feasible source oracle audit completed: %d source-rule rows.\n',height(summary));
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

