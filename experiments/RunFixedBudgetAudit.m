function results = RunFixedBudgetAudit
%RUNFIXEDBUDGETAUDIT 审计Adaptive是否只是固定更大Relocate预算
%   比较Fixed-5/15/25/35/40与Adaptive，使用相同8个动态场景和配对种子。

root=fileparts(fileparts(mfilename('fullpath'))); setup;
resultDir=fullfile(root,'results');
configs={
    'addition_instances','M1',9,'addition_M1_09.mat';
    'addition_instances','M1',10,'addition_M1_10.mat';
    'addition_stress_instances','Near',2,'stress_Near_02.mat';
    'addition_stress_instances','Near',4,'stress_Near_04.mat';
    'addition_stress_instances','Tight',1,'stress_Tight_01.mat';
    'addition_stress_instances','Far',1,'stress_Far_01.mat';
    'addition_stress_instances','Far',3,'stress_Far_03.mat';
    'addition_stress_instances','Far',5,'stress_Far_05.mat'};
fixedDepths=[5 15 25 35 40];
methods=["Fixed5","Fixed15","Fixed25","Fixed35","Fixed40","Adaptive"];
checkpoints=[500 1000 1500 2500]; seeds=1:5;
cfg.population=30; cfg.inertia=0.90; cfg.inertiaDamp=0.995;
cfg.c1=1.7; cfg.c2=1.7; cfg.maxBudget=2500;
cfg.localMinFE=5; cfg.localMaxFE=40; cfg.efficiencySmoothing=0.25;
rows=cell(0,1);

for scenarioIndex=1:size(configs,1)
    dataset=configs{scenarioIndex,1}; level=configs{scenarioIndex,2};
    instanceID=configs{scenarioIndex,3}; fileName=configs{scenarioIndex,4};
    loaded=load(fullfile(resultDir,dataset,fileName)); instance=loaded.instance;
    model=instance.model; cache=instance.cache; activeIDs=instance.activeIDs;
    referenceCost=instance.referenceCost; [previousSolution,~,~]=BuildPreEventSolution;
    for seedIndex=1:numel(seeds)
        algorithmSeed=720000+1000*instance.scenarioSeed+seeds(seedIndex);
        rng(algorithmSeed,'twister'); warmTimer=tic;
        [warmPositions,warmInfo]=BuildWarmStartPopulation(previousSolution, ...
            activeIDs,model,cache,cfg.population);
        warmTime=toc(warmTimer); warmFE=warmInfo.evaluations;
        for methodIndex=1:numel(methods)
            method=methods(methodIndex);
            if method=="Adaptive"
                options=struct('nPop',cfg.population,'maxFE',max(0,cfg.maxBudget-warmFE), ...
                    'w',cfg.inertia,'wdamp',cfg.inertiaDamp,'c1',cfg.c1,'c2',cfg.c2, ...
                    'localMinFE',cfg.localMinFE,'localMaxFE',cfg.localMaxFE, ...
                    'efficiencySmoothing',cfg.efficiencySmoothing);
                rng(algorithmSeed,'twister'); timer=tic;
                [solution,history,stats]=AdaptiveMemeticRoutingPSO(model,cache, ...
                    options,warmPositions);
                responseTime=warmTime+toc(timer); warmUsed=warmFE;
            else
                depth=fixedDepths(methodIndex);
                options=struct('enabled',true,'mode','global-relocate', ...
                    'localSearchFE',depth);
                perIterationFE=cfg.population+depth;
                available=max(0,cfg.maxBudget-warmFE);
                maxIt=max(0,floor(max(0,available-cfg.population)/perIterationFE));
                rng(algorithmSeed,'twister'); timer=tic;
                [solution,~,stats,history]=RoutingPSO(model,cache,maxIt, ...
                    cfg.population,cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2, ...
                    warmPositions,options);
                responseTime=warmTime+toc(timer); warmUsed=warmFE;
            end
            for checkpoint=checkpoints
                idx=find(warmUsed+history.FE<=checkpoint,1,'last');
                if isempty(idx), continue; end
                fitness=history.Cost(idx); distance=history.Distance(idx);
                late=history.Late(idx); feasible=history.IsFeasible(idx);
                actualFE=warmUsed+history.FE(idx); localFE=history.LocalSearchFE(idx);
                if isfield(history,'LocalBudget')
                    localBudget=history.LocalBudget(idx);
                elseif method=="Adaptive"
                    localBudget=NaN;
                else
                    localBudget=fixedDepths(methodIndex);
                end
                if feasible, gap=(distance-referenceCost)/max(abs(referenceCost),eps); else, gap=NaN; end
                rows{end+1,1}={string(dataset),string(level),instanceID, ...
                    instance.scenarioSeed,algorithmSeed,method,seedIndex,checkpoint, ...
                    actualFE,warmUsed,localFE,localBudget,responseTime,fitness, ...
                    distance,late,feasible,referenceCost,gap}; %#ok<AGROW>
            end
        end
    end
    fprintf('%s-%d completed.\n',level,instanceID);
end

summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'dataset','level','instance','scenario_seed','algorithm_seed','method', ...
    'seed_index','checkpoint_fe','actual_fe','warm_start_fe','local_search_fe', ...
    'local_budget','response_time','fitness','distance','total_late', ...
    'is_feasible','reference_cost','gap_to_reference'});
results.summary=summary; results.methods=methods; results.fixedDepths=fixedDepths;
results.checkpoints=checkpoints; results.cfg=cfg;
writetable(summary,fullfile(resultDir,'fixed_budget_audit_summary.csv'));
save(fullfile(resultDir,'fixed_budget_audit_result.mat'),'results','-v7.3');
fprintf('Fixed-budget audit completed: %d rows.\n',height(summary));
end
