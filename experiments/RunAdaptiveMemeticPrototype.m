function results = RunAdaptiveMemeticPrototype
%RUNADAPTIVEMEMETICPROTOTYPE 验证可行性状态与FE预算自适应模因PSO
%   比较：Repair、WarmPSO、固定Relocate、AdaptiveMemeticPSO。

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
strategies=["Repair","WarmPSO","WarmPSO_FixedRelocate","AdaptiveMemeticPSO"];
checkpoints=[500 1000 1500 2500]; seeds=1:10;
cfg.population=30; cfg.inertia=0.90; cfg.inertiaDamp=0.995;
cfg.c1=1.7; cfg.c2=1.7; cfg.localSearchFE=25; cfg.maxBudget=2500;
cfg.localMinFE=5; cfg.localMaxFE=40; cfg.efficiencySmoothing=0.25;
cfg.feasibilityAlpha=0.85; rows=cell(0,1);

for scenarioIndex=1:size(configs,1)
    dataset=configs{scenarioIndex,1}; level=configs{scenarioIndex,2};
    instanceID=configs{scenarioIndex,3}; fileName=configs{scenarioIndex,4};
    loaded=load(fullfile(resultDir,dataset,fileName)); instance=loaded.instance;
    model=instance.model; cache=instance.cache; activeIDs=instance.activeIDs;
    event=instance.event; referenceCost=instance.referenceCost;
    [previousSolution,~,~]=BuildPreEventSolution;
    for seedIndex=1:numel(seeds)
        algorithmSeed=710000+1000*instance.scenarioSeed+seeds(seedIndex);
        rng(algorithmSeed,'twister'); warmTimer=tic;
        [warmPositions,warmInfo]=BuildWarmStartPopulation(previousSolution, ...
            activeIDs,model,cache,cfg.population);
        warmTime=toc(warmTimer); warmFE=warmInfo.evaluations;
        repairCost=NaN;
        for strategyIndex=1:numel(strategies)
            strategy=strategies(strategyIndex);
            if strategy=="Repair"
                timer=tic;
                [solution,repairInfo]=LocalRepair(previousSolution,activeIDs, ...
                    model,cache,event.type,event.customerIDs);
                runData.solution=solution; runData.history=[];
                runData.stats.functionEvaluations=repairInfo.functionEvaluations;
                runData.stats.firstFeasibleEvaluation=NaN;
                runData.stats.localSearchFE=0; runData.stats.feasibilityModeFE=0;
                runData.stats.qualityModeFE=0; runData.stats.localSearchImprovementCount=0;
                runData.stats.meanLocalBudget=0; runData.warmFE=0;
                runData.responseTime=toc(timer); runData.totalFE=repairInfo.functionEvaluations;
                repairCost=solution.Cost;
            elseif strategy=="WarmPSO" || strategy=="WarmPSO_FixedRelocate"
                searchOptions=struct('enabled',false,'mode','none','localSearchFE',0);
                perIterationFE=cfg.population;
                if strategy=="WarmPSO_FixedRelocate"
                    searchOptions.enabled=true; searchOptions.mode='global-relocate';
                    searchOptions.localSearchFE=cfg.localSearchFE;
                    perIterationFE=cfg.population+cfg.localSearchFE;
                end
                available=max(0,cfg.maxBudget-warmFE);
                maxIt=max(0,floor(max(0,available-cfg.population)/perIterationFE));
                rng(algorithmSeed,'twister'); timer=tic;
                [solution,~,stats,history]=RoutingPSO(model,cache,maxIt, ...
                    cfg.population,cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2, ...
                    warmPositions,searchOptions);
                runData.solution=solution; runData.history=history; runData.stats=stats;
                runData.warmFE=warmFE; runData.responseTime=warmTime+toc(timer);
                runData.totalFE=warmFE+stats.functionEvaluations;
            else
                options=struct('nPop',cfg.population,'maxFE',max(0,cfg.maxBudget-warmFE), ...
                    'w',cfg.inertia,'wdamp',cfg.inertiaDamp,'c1',cfg.c1,'c2',cfg.c2, ...
                    'localMinFE',cfg.localMinFE,'localMaxFE',cfg.localMaxFE, ...
                    'efficiencySmoothing',cfg.efficiencySmoothing, ...
                    'feasibilityAlpha',cfg.feasibilityAlpha,'silent',true);
                rng(algorithmSeed,'twister'); timer=tic;
                [solution,history,stats]=AdaptiveMemeticRoutingPSO(model,cache, ...
                    options,warmPositions);
                runData.solution=solution; runData.history=history; runData.stats=stats;
                runData.warmFE=warmFE; runData.responseTime=warmTime+toc(timer);
                runData.totalFE=warmFE+stats.functionEvaluations;
            end

            for checkpoint=checkpoints
                if strategy=="Repair"
                    solution=runData.solution; actualFE=runData.totalFE;
                    fitness=solution.Cost; distance=solution.Detail.distance;
                    late=solution.Detail.totalLate; feasible=solution.Detail.isFeasible;
                    localFE=0; localBudget=0; psoEff=NaN; localEff=NaN; mode="repair"; firstFeasible=NaN;
                    feasFE=0; qualityFE=0; improve100=NaN; meanBudget=0;
                else
                    index=find(runData.warmFE+runData.history.FE<=checkpoint,1,'last');
                    if isempty(index), continue; end
                    h=runData.history; fitness=h.Cost(index); distance=h.Distance(index);
                    late=h.Late(index); feasible=h.IsFeasible(index);
                    actualFE=runData.warmFE+h.FE(index); localFE=h.LocalSearchFE(index);
                    if isfield(h,'LocalBudget'), localBudget=h.LocalBudget(index); else, localBudget=cfg.localSearchFE; end
                    if isfield(h,'PSOImprovementPerFE'), psoEff=h.PSOImprovementPerFE(index); else, psoEff=NaN; end
                    if isfield(h,'LocalImprovementPerFE'), localEff=h.LocalImprovementPerFE(index); else, localEff=NaN; end
                    if isfield(h,'Mode'), mode=string(h.Mode(index)); else, mode=string(strategy); end
                    firstFeasible=runData.warmFE+runData.stats.firstFeasibleEvaluation;
                    if isinf(runData.stats.firstFeasibleEvaluation), firstFeasible=NaN; end
                    if isfield(runData.stats,'feasibilityModeFE'), feasFE=runData.stats.feasibilityModeFE; else, feasFE=0; end
                    if isfield(runData.stats,'qualityModeFE'), qualityFE=runData.stats.qualityModeFE; else, qualityFE=runData.stats.localSearchFE; end
                    if isfield(runData.stats,'localSearchImprovementCount'), improve100=100*runData.stats.localSearchImprovementCount/max(runData.stats.localSearchFE,1); else, improve100=NaN; end
                    if isfield(runData.stats,'meanLocalBudget'), meanBudget=runData.stats.meanLocalBudget; else, meanBudget=cfg.localSearchFE; end
                end
                if feasible, gap=(distance-referenceCost)/max(abs(referenceCost),eps); else, gap=NaN; end
                overtake=false; if strategy~="Repair", overtake=fitness<repairCost; end
                rows{end+1,1}={string(dataset),string(level),instanceID, ...
                    instance.scenarioSeed,algorithmSeed,string(event.type), ...
                    numel(event.customerIDs),strategy,seedIndex,checkpoint, ...
                    actualFE,runData.warmFE,localFE,localBudget,psoEff,localEff,runData.responseTime,fitness, ...
                    distance,late,feasible,referenceCost,gap,overtake,firstFeasible, ...
                    feasFE,qualityFE,improve100,meanBudget,mode}; %#ok<AGROW>
            end
        end
    end
    fprintf('%s-%d completed.\n',level,instanceID);
end

summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'dataset','level','instance','scenario_seed','algorithm_seed','event_type', ...
    'event_size','strategy','seed_index','checkpoint_fe','actual_fe', ...
    'warm_start_fe','local_search_fe','local_budget','pso_improvement_per_fe', ...
    'local_improvement_per_fe','response_time','fitness','distance', ...
    'total_late','is_feasible','reference_cost','gap_to_reference', ...
    'overtake_repair','first_feasible_fe','feasibility_mode_fe', ...
    'quality_mode_fe','improvement_per_100_local_fe','mean_local_budget','mode'});
results.summary=summary; results.configs=configs; results.strategies=strategies;
results.checkpoints=checkpoints; results.cfg=cfg;
writetable(summary,fullfile(resultDir,'adaptive_memetic_prototype_summary.csv'));
save(fullfile(resultDir,'adaptive_memetic_prototype_result.mat'),'results','-v7.3');
PlotAdaptivePrototype(summary,resultDir);
fprintf('Adaptive memetic prototype completed: %d rows.\n',height(summary));
end

function PlotAdaptivePrototype(summary,resultDir)
figure('Visible','off','Color','w'); tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
strategies=unique(summary.strategy,'stable');
nexttile; hold on;
for k=1:numel(strategies)
    subset=summary(summary.strategy==strategies(k),:); groups=findgroups(subset.strategy,subset.checkpoint_fe);
    means=splitapply(@mean,subset.total_late,groups); cps=splitapply(@(x)x(1),subset.checkpoint_fe,groups);
    plot(cps,means,'-o','LineWidth',1.3,'DisplayName',strategies(k));
end
grid on; xlabel('Strict FE checkpoint'); ylabel('Mean total late'); legend('Location','best');
title('Fixed versus feasibility-state adaptive memetic search');
nexttile; hold on;
for k=1:numel(strategies)
    subset=summary(summary.strategy==strategies(k) & ~isnan(summary.gap_to_reference),:);
    if isempty(subset), continue; end
    groups=findgroups(subset.strategy,subset.checkpoint_fe);
    means=splitapply(@mean,subset.gap_to_reference,groups); cps=splitapply(@(x)x(1),subset.checkpoint_fe,groups);
    plot(cps,100*means,'-o','LineWidth',1.3,'DisplayName',strategies(k));
end
grid on; xlabel('Strict FE checkpoint'); ylabel('Mean gap to reference (%)'); legend('Location','best');
title('Feasible-run quality trajectory');
exportgraphics(gcf,fullfile(resultDir,'adaptive_memetic_prototype_curves.png'),'Resolution',180); close(gcf);
end

