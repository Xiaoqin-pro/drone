function results = RunImpactGuidedRelocatePrototype
%RUNIMPACTGUIDEDRELOCATEPROTOTYPE 比较Global和Impact-guided Relocate
%   主线：Repair、WarmPSO、WarmPSO+GlobalRelocate、WarmPSO+ImpactRelocate。

root=fileparts(fileparts(mfilename('fullpath')));
setup;
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
strategies=["Repair","WarmPSO","WarmPSO_GlobalRelocate", ...
    "WarmPSO_ImpactRelocate"];
checkpoints=[500 1000 1500 2500];
seeds=1:10;
cfg.population=30; cfg.inertia=0.90; cfg.inertiaDamp=0.995;
cfg.c1=1.7; cfg.c2=1.7; cfg.localSearchFE=25;
cfg.maxBudget=2500; cfg.impactAlpha=0.7;
rows=cell(0,1);

for scenarioIndex=1:size(configs,1)
    dataset=configs{scenarioIndex,1}; level=configs{scenarioIndex,2};
    instanceID=configs{scenarioIndex,3}; fileName=configs{scenarioIndex,4};
    loaded=load(fullfile(resultDir,dataset,fileName)); instance=loaded.instance;
    model=instance.model; cache=instance.cache; activeIDs=instance.activeIDs;
    event=instance.event; referenceCost=instance.referenceCost;

    % 与动态基准生成时保持一致的事件前旧计划。
    [previousSolution,~,~]=BuildPreEventSolution;

    for seedIndex=1:numel(seeds)
        algorithmSeed=700000+1000*instance.scenarioSeed+seeds(seedIndex);
        rng(algorithmSeed,'twister');
        warmTimer=tic;
        [warmPositions,warmInfo]=BuildWarmStartPopulation( ...
            previousSolution,activeIDs,model,cache,cfg.population);
        warmTime=toc(warmTimer); warmFE=warmInfo.evaluations;

        % 影响画像只在Impact方法中计算，并显式计入总FE。
        impactTimer=tic;
        profile=ComputeImpactProfile(previousSolution.Detail.routeIDs, ...
            activeIDs,event.customerIDs,model,cache);
        impactTime=toc(impactTimer);
        impactIDs=profile.impactIDs;
        impactScores=profile.impactScores;

        for strategyIndex=1:numel(strategies)
            strategy=strategies(strategyIndex);
            if strategy=="Repair"
                timer=tic;
                [solution,repairInfo]=LocalRepair(previousSolution,activeIDs, ...
                    model,cache,event.type,event.customerIDs);
                runData.solution=solution; runData.history=[];
                runData.stats.functionEvaluations=repairInfo.functionEvaluations;
                runData.stats.firstFeasibleEvaluation=double(repairInfo.isFeasible);
                runData.warmFE=0; runData.impactFE=0;
                runData.responseTime=toc(timer); runData.impactTime=0;
                runData.totalFE=repairInfo.functionEvaluations;
                repairCost=solution.Cost;
            else
                searchOptions=struct('enabled',false,'mode','none', ...
                    'localSearchFE',0,'impactAlpha',cfg.impactAlpha, ...
                    'impactIDs',activeIDs,'impactScores',zeros(size(activeIDs)));
                impactFE=0; preprocessingTime=0;
                if strategy=="WarmPSO_GlobalRelocate"
                    searchOptions.enabled=true;
                    searchOptions.mode='global-relocate';
                    searchOptions.localSearchFE=cfg.localSearchFE;
                elseif strategy=="WarmPSO_ImpactRelocate"
                    searchOptions.enabled=true;
                    searchOptions.mode='impact-relocate';
                    searchOptions.localSearchFE=cfg.localSearchFE;
                    searchOptions.impactIDs=impactIDs;
                    searchOptions.impactScores=impactScores;
                    impactFE=profile.preprocessFE;
                    preprocessingTime=impactTime;
                end
                if strategy=="WarmPSO"
                    perIterationFE=cfg.population;
                else
                    perIterationFE=cfg.population+cfg.localSearchFE;
                end
                available=max(0,cfg.maxBudget-warmFE-impactFE);
                maxIt=max(0,floor(max(0,available-cfg.population)/perIterationFE));
                rng(algorithmSeed,'twister'); timer=tic;
                [solution,~,stats,history]=RoutingPSO(model,cache,maxIt, ...
                    cfg.population,cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2, ...
                    warmPositions,searchOptions);
                runData.solution=solution; runData.history=history;
                runData.stats=stats; runData.warmFE=warmFE;
                runData.impactFE=impactFE;
                runData.responseTime=warmTime+preprocessingTime+toc(timer);
                runData.impactTime=impactTime;
                runData.totalFE=warmFE+impactFE+stats.functionEvaluations;
            end

            for checkpoint=checkpoints
                if strategy=="Repair"
                    solution=runData.solution; actualFE=runData.totalFE;
                    fitness=solution.Cost; distance=solution.Detail.distance;
                    late=solution.Detail.totalLate; feasible=solution.Detail.isFeasible;
                    localFE=0; firstFeasible=NaN;
                    impactPreprocessFE=0; priorityFE=0; globalFE=0;
                    improvementPer100=NaN; meanSourceRank=NaN;
                else
                    history=runData.history;
                    index=find(runData.warmFE+runData.impactFE+history.FE<=checkpoint,1,'last');
                    if isempty(index), continue; end
                    fitness=history.Cost(index); distance=history.Distance(index);
                    late=history.Late(index); feasible=history.IsFeasible(index);
                    actualFE=runData.warmFE+runData.impactFE+history.FE(index);
                    localFE=history.LocalSearchFE(index);
                    impactPreprocessFE=runData.impactFE;
                    priorityFE=NaN; globalFE=NaN;
                    if strategy=="WarmPSO_ImpactRelocate"
                        priorityFE=runData.stats.priorityFE;
                        globalFE=runData.stats.globalFE;
                    end
                    firstFeasible=runData.warmFE+runData.impactFE+runData.stats.firstFeasibleEvaluation;
                    if isinf(runData.stats.firstFeasibleEvaluation), firstFeasible=NaN; end
                    improvementPer100=100*runData.stats.localSearchImprovementCount/ ...
                        max(runData.stats.localSearchFE,1);
                    if isempty(runData.stats.acceptedSourceRanks)
                        meanSourceRank=NaN;
                    else
                        meanSourceRank=mean(runData.stats.acceptedSourceRanks);
                    end
                end
                if feasible
                    gap=(distance-referenceCost)/max(abs(referenceCost),eps);
                else
                    gap=NaN;
                end
                overtake=false;
                if strategy~="Repair", overtake=fitness<repairCost; end
                rows{end+1,1}={string(dataset),string(level),instanceID, ...
                    instance.scenarioSeed,algorithmSeed,string(event.type), ...
                    numel(event.customerIDs),strategy,seedIndex,checkpoint, ...
                    actualFE,runData.warmFE,runData.impactFE,localFE, ...
                    runData.responseTime,fitness,distance,late,feasible, ...
                    referenceCost,gap,overtake,firstFeasible,priorityFE,globalFE, ...
                    improvementPer100,meanSourceRank}; %#ok<AGROW>
            end
        end
    end
    fprintf('%s-%d completed.\n',level,instanceID);
end

summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'dataset','level','instance','scenario_seed','algorithm_seed', ...
    'event_type','event_size','strategy','seed_index','checkpoint_fe', ...
    'actual_fe','warm_start_fe','impact_preprocess_fe','local_search_fe', ...
    'response_time','fitness','distance','total_late','is_feasible', ...
    'reference_cost','gap_to_reference','overtake_repair','first_feasible_fe', ...
    'impact_priority_fe','impact_global_fe','improvement_per_100_local_fe', ...
    'mean_accepted_source_rank'});
results.summary=summary; results.configs=configs; results.strategies=strategies;
results.checkpoints=checkpoints; results.cfg=cfg;
writetable(summary,fullfile(resultDir,'impact_guided_relocate_summary.csv'));
save(fullfile(resultDir,'impact_guided_relocate_result.mat'),'results','-v7.3');
PlotImpactPrototype(summary,resultDir);
fprintf('Impact-guided Relocate prototype completed: %d rows.\n',height(summary));
end

function PlotImpactPrototype(summary,resultDir)
figure('Visible','off','Color','w'); tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
strategies=unique(summary.strategy,'stable');
nexttile; hold on;
for k=1:numel(strategies)
    subset=summary(summary.strategy==strategies(k),:);
    groups=findgroups(subset.strategy,subset.checkpoint_fe);
    means=splitapply(@mean,subset.total_late,groups);
    cps=splitapply(@(x)x(1),subset.checkpoint_fe,groups);
    plot(cps,means,'-o','LineWidth',1.3,'DisplayName',strategies(k));
end
grid on; xlabel('Strict FE checkpoint'); ylabel('Mean total late');
legend('Location','best'); title('Global vs impact-guided Relocate');
nexttile; hold on;
for k=1:numel(strategies)
    subset=summary(summary.strategy==strategies(k) & ~isnan(summary.gap_to_reference),:);
    if isempty(subset), continue; end
    groups=findgroups(subset.strategy,subset.checkpoint_fe);
    means=splitapply(@mean,subset.gap_to_reference,groups);
    cps=splitapply(@(x)x(1),subset.checkpoint_fe,groups);
    plot(cps,100*means,'-o','LineWidth',1.3,'DisplayName',strategies(k));
end
grid on; xlabel('Strict FE checkpoint'); ylabel('Mean gap to reference (%)');
legend('Location','best'); title('Feasible-run gap trajectory');
exportgraphics(gcf,fullfile(resultDir,'impact_guided_relocate_curves.png'),'Resolution',180); close(gcf);
end
