function results = RunDynamicFBCMPSOPrototype
%RUNDYNAMICFBCMPSOPROTOTYPE 动态三维UAV上验证最终FB-CMPSO迁移
%   Repair / WarmPSO / FB-CMPSO，使用与TSPTW一致的边界恢复+VND结构。

root=fileparts(fileparts(mfilename('fullpath'))); setup; resultDir=fullfile(root,'results');
configs={
    'addition_instances','M1',9,'addition_M1_09.mat';
    'addition_instances','M1',10,'addition_M1_10.mat';
    'addition_stress_instances','Near',2,'stress_Near_02.mat';
    'addition_stress_instances','Near',4,'stress_Near_04.mat';
    'addition_stress_instances','Tight',1,'stress_Tight_01.mat';
    'addition_stress_instances','Far',1,'stress_Far_01.mat';
    'addition_stress_instances','Far',3,'stress_Far_03.mat';
    'addition_stress_instances','Far',5,'stress_Far_05.mat'};
methods=["Repair","WarmPSO","FB-CMPSO"]; checkpoints=[500 1000 1500 2500]; seeds=1:10;
cfg.population=30; cfg.inertia=.90; cfg.inertiaDamp=.995; cfg.c1=1.7; cfg.c2=1.7; cfg.maxBudget=2500;
rows=cell(0,1);
for si=1:size(configs,1)
    dataset=configs{si,1}; level=configs{si,2}; instanceID=configs{si,3}; fileName=configs{si,4};
    loaded=load(fullfile(resultDir,dataset,fileName)); instance=loaded.instance;
    model=instance.model; cache=instance.cache; activeIDs=instance.activeIDs; event=instance.event;
    referenceCost=instance.referenceCost;
    [previousSolution,~,~]=BuildPreEventSolution;
    for seedIndex=1:numel(seeds)
        algorithmSeed=1300000+1000*instance.scenarioSeed+seedIndex;
        rng(algorithmSeed,'twister'); tWarm=tic;
        [warmPositions,warmInfo]=BuildWarmStartPopulation(previousSolution,activeIDs,model,cache,cfg.population);
        warmTime=toc(tWarm); warmFE=warmInfo.evaluations;
        repairCost=NaN;
        for mi=1:numel(methods)
            method=methods(mi);
            if method=="Repair"
                timer=tic; [solution,repairInfo]=LocalRepair(previousSolution,activeIDs,model,cache,event.type,event.customerIDs);
                run.solution=solution; run.history=[]; run.stats.functionEvaluations=repairInfo.functionEvaluations;
                run.stats.firstFeasibleEvaluation=NaN; run.stats.phaseSwitchFE=NaN; run.warmFE=0;
                run.responseTime=toc(timer); run.totalFE=repairInfo.functionEvaluations; repairCost=solution.Cost;
            else
                search=struct('enabled',false,'mode','none','localSearchFE',0,'restorationFE', ...
                    max(1,numel(activeIDs)-1),'intensificationFE',3*max(1,numel(activeIDs)-1), ...
                    'maxFE',cfg.maxBudget-warmFE);
                if method=="FB-CMPSO"
                    search.enabled=true; search.mode='boundary-vnd';
                    search.localSearchFE=search.intensificationFE;
                    perIterationFE=cfg.population+search.intensificationFE;
                else
                    perIterationFE=cfg.population;
                end
                available=max(0,cfg.maxBudget-warmFE);
                maxIt=max(0,floor(max(0,available-cfg.population)/perIterationFE));
                rng(algorithmSeed,'twister'); timer=tic;
                [solution,~,stats,history]=RoutingPSO(model,cache,maxIt,cfg.population, ...
                    cfg.inertia,cfg.inertiaDamp,cfg.c1,cfg.c2,warmPositions,search);
                run.solution=solution; run.history=history; run.stats=stats; run.warmFE=warmFE;
                run.responseTime=warmTime+toc(timer); run.totalFE=warmFE+stats.functionEvaluations;
            end
            for checkpoint=checkpoints
                if method=="Repair"
                    actualFE=run.totalFE; solution=run.solution; fitness=solution.Cost; distance=solution.Detail.distance;
                    late=solution.Detail.totalLate; feasible=solution.Detail.isFeasible; firstFE=NaN; phaseFE=NaN; phaseGain=NaN; localFE=0;
                else
                    idx=find(run.warmFE+run.history.FE<=checkpoint,1,'last'); if isempty(idx), continue; end
                    history=run.history; actualFE=run.warmFE+history.FE(idx); fitness=history.Cost(idx);
                    distance=history.Distance(idx); late=history.Late(idx); feasible=history.IsFeasible(idx); localFE=history.LocalSearchFE(idx);
                    firstFE=run.warmFE+run.stats.firstFeasibleEvaluation; if isinf(run.stats.firstFeasibleEvaluation), firstFE=NaN; end
                    phaseFE=run.warmFE+run.stats.phaseSwitchFE; if isinf(run.stats.phaseSwitchFE), phaseFE=NaN; end
                    if isfield(run.stats,'phaseGain'), phaseGain=run.stats.phaseGain; else, phaseGain=NaN; end
                end
                if feasible && isfinite(referenceCost), gap=(distance-referenceCost)/max(abs(referenceCost),eps); else, gap=NaN; end
                overtake=false; if method~="Repair", overtake=fitness<repairCost; end
                rows{end+1,1}={string(dataset),string(level),instanceID,instance.scenarioSeed, ...
                    algorithmSeed,string(event.type),numel(event.customerIDs),method,seedIndex,checkpoint, ...
                    actualFE,run.warmFE,localFE,run.responseTime,fitness,distance,late,feasible, ...
                    referenceCost,gap,overtake,firstFE,phaseFE,phaseGain}; %#ok<AGROW>
            end
        end
    end
    fprintf('%s-%d completed.\n',level,instanceID);
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'dataset','level','instance','scenario_seed','algorithm_seed','event_type','event_size','strategy', ...
    'seed_index','checkpoint_fe','actual_fe','warm_start_fe','local_search_fe','response_time','fitness', ...
    'distance','total_late','is_feasible','reference_cost','gap_to_reference','overtake_repair', ...
    'first_feasible_fe','phase_switch_fe','phase_gain'});
results.summary=summary; results.configs=configs; results.methods=methods; results.checkpoints=checkpoints;
writetable(summary,fullfile(resultDir,'dynamic_fbcmps0_summary.csv'));
save(fullfile(resultDir,'dynamic_fbcmps0_result.mat'),'results','-v7.3');
fprintf('Dynamic FB-CMPSO prototype completed: %d rows.\n',height(summary));
end

