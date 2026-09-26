function results = RunDynamicMemeticPrototype
%RUNDYNAMICMEMETICPROTOTYPE 回到三维动态UAV主线的最小验证实验
%   比较：Repair、WarmPSO、WarmPSO+Relocate。
%   Relocate直接调用EvaluateSchedule，因此每次移动都会重新评价三维安全航段、到达时间和时间窗。

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
strategies=["Repair","WarmPSO","WarmPSO_Relocate"];
checkpoints=[500 1000 1500 2500];
seeds=1:10;
cfg.population=30;
cfg.inertia=0.90;
cfg.inertiaDamp=0.995;
cfg.c1=1.7;
cfg.c2=1.7;
cfg.localSearchFE=25;
cfg.maxBudget=2500;
rows=cell(0,1);
runRecords=cell(size(configs,1),numel(seeds),numel(strategies));

for scenarioIndex=1:size(configs,1)
    dataset=configs{scenarioIndex,1};
    level=configs{scenarioIndex,2};
    instanceID=configs{scenarioIndex,3};
    fileName=configs{scenarioIndex,4};
    loaded=load(fullfile(resultDir,dataset,fileName));
    instance=loaded.instance;
    model=instance.model; cache=instance.cache; activeIDs=instance.activeIDs;
    event=instance.event;
    if isfield(instance,'referenceCost'), referenceCost=instance.referenceCost; else, referenceCost=NaN; end

    % 事件前初始路线只构造一次，不进入事件后的比较预算。
    baseModel=CreateModel(); baseModel.cfg.silent=true;
    initialIDs=baseModel.customerIDs(:)';
    initialCache=BuildLegCache(baseModel,initialIDs,baseModel.homePosition);
    rng(880000+instance.scenarioSeed,'twister');
    [initialSolution,~,~]=RoutingPSO(baseModel,initialCache,50,30, ...
        cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2,[]);

    for seedIndex=1:numel(seeds)
        algorithmSeed=700000+1000*instance.scenarioSeed+seeds(seedIndex);
        rng(algorithmSeed,'twister');
        warmTimer=tic;
        [warmPositions,warmInfo]=BuildWarmStartPopulation( ...
            initialSolution,activeIDs,model,cache,cfg.population);
        warmTime=toc(warmTimer);
        warmFE=warmInfo.evaluations;

        strategyRuns=cell(1,numel(strategies));
        repairCost=NaN;
        for strategyIndex=1:numel(strategies)
            strategy=strategies(strategyIndex);
            if strategy=="Repair"
                timer=tic;
                [solution,repairInfo]=LocalRepair(initialSolution,activeIDs, ...
                    model,cache,event.type,event.customerIDs);
                elapsed=toc(timer);
                runData.solution=solution;
                runData.history=[];
                runData.stats.functionEvaluations=repairInfo.functionEvaluations;
                runData.warmFE=0;
                runData.responseTime=elapsed;
                runData.totalFE=repairInfo.functionEvaluations;
                repairCost=solution.Cost;
            else
                if strategy=="WarmPSO"
                    initialPositions=warmPositions;
                    searchOptions=struct('enabled',false,'mode','none', ...
                        'localSearchFE',0,'priorityIDs',activeIDs);
                    perIterationFE=cfg.population;
                else
                    initialPositions=warmPositions;
                    searchOptions=struct('enabled',true,'mode','relocate', ...
                        'localSearchFE',cfg.localSearchFE,'priorityIDs',activeIDs);
                    perIterationFE=cfg.population+cfg.localSearchFE;
                end
                available=max(0,cfg.maxBudget-warmFE);
                maxIt=max(0,floor(max(0,available-cfg.population)/perIterationFE));
                rng(algorithmSeed,'twister');
                timer=tic;
                [solution,~,stats,history]=RoutingPSO(model,cache,maxIt, ...
                    cfg.population,cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2, ...
                    initialPositions,searchOptions);
                elapsed=toc(timer);
                runData.solution=solution;
                runData.history=history;
                runData.stats=stats;
                runData.warmFE=warmFE;
                runData.responseTime=warmTime+elapsed;
                runData.totalFE=warmFE+stats.functionEvaluations;
            end
            strategyRuns{strategyIndex}=runData;
            runRecords{scenarioIndex,seedIndex,strategyIndex}=runData;
        end

        for strategyIndex=1:numel(strategies)
            strategy=strategies(strategyIndex);
            runData=strategyRuns{strategyIndex};
            for checkpoint=checkpoints
                if strategy=="Repair"
                    solution=runData.solution;
                    actualFE=runData.totalFE;
                    fitness=solution.Cost;
                    distance=solution.Detail.distance;
                    late=solution.Detail.totalLate;
                    feasible=solution.Detail.isFeasible;
                    localFE=0;
                    firstFeasible=NaN;
                else
                    history=runData.history;
                    index=find(runData.warmFE+history.FE<=checkpoint,1,'last');
                    if isempty(index), continue; end
                    fitness=history.Cost(index);
                    distance=history.Distance(index);
                    late=history.Late(index);
                    feasible=history.IsFeasible(index);
                    actualFE=runData.warmFE+history.FE(index);
                    localFE=history.LocalSearchFE(index);
                    firstFeasible=runData.warmFE+runData.stats.firstFeasibleEvaluation;
                    if isinf(runData.stats.firstFeasibleEvaluation), firstFeasible=NaN; end
                end
                if feasible && isfinite(referenceCost)
                    gap=(distance-referenceCost)/max(abs(referenceCost),eps);
                else
                    gap=NaN;
                end
                if strategy=="Repair"
                    overtake=false;
                else
                    overtake=fitness<repairCost;
                end
                rows{end+1,1}={string(dataset),string(level),instanceID, ...
                    instance.scenarioSeed,algorithmSeed,string(event.type), ...
                    numel(event.customerIDs),string(strategy),seedIndex, ...
                    checkpoint,actualFE,runData.warmFE,localFE, ...
                    runData.responseTime,fitness,distance,late,feasible, ...
                    referenceCost,gap,overtake,firstFeasible}; %#ok<AGROW>
            end
        end
    end
    fprintf('%s-%d completed.\n',level,instanceID);
end

summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'dataset','level','instance','scenario_seed','algorithm_seed', ...
    'event_type','event_size','strategy','seed_index','checkpoint_fe', ...
    'actual_fe','warm_start_fe','local_search_fe','response_time', ...
    'fitness','distance','total_late','is_feasible','reference_cost', ...
    'gap_to_reference','overtake_repair','first_feasible_fe'});
results.summary=summary;
results.configs=configs;
results.strategies=strategies;
results.checkpoints=checkpoints;
results.cfg=cfg;
writetable(summary,fullfile(resultDir,'dynamic_memetic_prototype_summary.csv'));
save(fullfile(resultDir,'dynamic_memetic_prototype_result.mat'),'results','-v7.3');
PlotDynamicPrototype(summary,resultDir);
fprintf('Dynamic memetic prototype completed: %d rows.\n',height(summary));
end

function PlotDynamicPrototype(summary,resultDir)
figure('Visible','off','Color','w');
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
nexttile; hold on;
strategies=unique(summary.strategy,'stable');
for k=1:numel(strategies)
    subset=summary(summary.strategy==strategies(k) & ~isnan(summary.total_late),:);
    groups=findgroups(subset.strategy,subset.checkpoint_fe);
    means=splitapply(@mean,subset.total_late,groups);
    cps=splitapply(@(x)x(1),subset.checkpoint_fe,groups);
    plot(cps,means,'-o','LineWidth',1.3,'DisplayName',strategies(k));
end
grid on; xlabel('Strict FE checkpoint'); ylabel('Mean total late');
legend('Location','best'); title('Dynamic 3D UAV: lateness trajectory');
nexttile; hold on;
for k=1:numel(strategies)
    subset=summary(summary.strategy==strategies(k) & ~isnan(summary.gap_to_reference),:);
    if isempty(subset), continue; end
    groups=findgroups(subset.strategy,subset.checkpoint_fe);
    means=splitapply(@mean,subset.gap_to_reference,groups);
    cps=splitapply(@(x)x(1),subset.checkpoint_fe,groups);
    plot(cps,100*means,'-o','LineWidth',1.3,'DisplayName',strategies(k));
end
grid on; xlabel('Strict FE checkpoint'); ylabel('Mean gap to witness (%)');
legend('Location','best'); title('Feasible-run gap trajectory');
exportgraphics(gcf,fullfile(resultDir,'dynamic_memetic_prototype_curves.png'),'Resolution',180);
close(gcf);
end
