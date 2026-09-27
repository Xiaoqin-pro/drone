function results=RunFinalDynamicUAV(nRuns,instanceDir,files,outDir)
%RUNFINALDYNAMICUAV 最终三维地形模型的动态重规划配对实验
%   独立输出，不覆盖legacy实验结果。nRuns=1仅作sanity，正式为30。
%   可通过 instanceDir/files/outDir 运行替代的困难实例集。

if nargin<1 || isempty(nRuns), nRuns=30; end
root=fileparts(fileparts(mfilename('fullpath'))); setup;
if nargin<2 || isempty(instanceDir), instanceDir=fullfile(root,'results','final_local_terrain_instances'); end
if nargin<3 || isempty(files)
    files=["addition_M1_09.mat","addition_M1_10.mat","stress_Near_02.mat", ...
        "stress_Near_04.mat","stress_Tight_01.mat","stress_Far_01.mat", ...
        "stress_Far_03.mat","stress_Far_05.mat"];
end
if nargin<4 || isempty(outDir), outDir=fullfile(root,'results','final_local_terrain'); end
if nRuns<30, tag='sanity'; else, tag='formal'; end
if contains(lower(outDir),'hard'), tag=[tag '_hard']; end
if ~isfolder(outDir), mkdir(outDir); end
methods=["Repair","WarmPSO","AlwaysVND","FB-CMPSO"];
checkpoints=[500 1000 1500 2500]; maxFE=max(checkpoints); nPop=30;
[previous,baseModel,~]=BuildPreEventSolution('local-terrain');
rows=cell(0,1); legRows=cell(0,1);
for scenario=1:numel(files)
    loaded=load(fullfile(instanceDir,files(scenario))); instance=loaded.instance;
    model=instance.model; cache=instance.cache; activeIDs=instance.activeIDs;
    actualState=ExecuteUntilEvent(previous,baseModel,instance.event.time);
    assert(norm(actualState.position-instance.state.position)<1e-8, '事件状态不匹配');
    assert(isequal(sort(actualState.completedIDs),sort(instance.state.completedIDs)), '已完成订单不匹配');
    [~,witness]=EvaluateSchedule(instance.referenceRoute,model,cache);
    assert(witness.isFeasible,'witness必须可行');
    legs=cache.legs; total=0; detour=0; unsafe=0; minClear=inf;
    for a=1:numel(legs)
        if isempty(legs{a}), continue; end
        total=total+1; detour=detour+(size(legs{a}.points,1)>2);
        unsafe=unsafe+~legs{a}.isFeasible; minClear=min(minClear,legs{a}.minClearance);
    end
    legRows{end+1,1}={files(scenario),total,detour,unsafe,minClear, ...
        min(model.customerXYZ(:,3)),max(model.customerXYZ(:,3))}; %#ok<AGROW>
    for run=1:nRuns
        algorithmSeed=1400000+1000*instance.scenarioSeed+run;
        rng(algorithmSeed,'twister'); tWarm=tic;
        [warmPositions,warmInfo]=BuildWarmStartPopulation(previous,activeIDs,model,cache,nPop);
        warmTime=toc(tWarm); warmFE=warmInfo.evaluations;
        rng(algorithmSeed,'twister'); tRepair=tic;
        [repair,repairInfo]=LocalRepair(previous,activeIDs,model,cache,instance.event.type,instance.event.customerIDs);
        repairTime=toc(tRepair);
        for method=methods
            if method=="Repair"
                for cp=checkpoints
                    rows{end+1,1}={files(scenario),instance.scenarioSeed,algorithmSeed, ...
                        method,cp,repairInfo.functionEvaluations,0,0,0, ...
                        repairTime,repair.Cost,repair.Detail.distance,repair.Detail.totalLate, ...
                        repair.Detail.totalObstacleViolation,repair.Detail.totalTerrainViolation, ...
                        repair.Detail.isFeasible,NaN,NaN,NaN,NaN,NaN,NaN,NaN, ...
                        instance.referenceCost,repair.Cost<instance.referenceCost}; %#ok<AGROW>
                end
                continue;
            end
            search=struct('enabled',false,'mode','none','localSearchFE',0, ...
                'restorationFE',max(1,numel(activeIDs)-1), ...
                'intensificationFE',3*max(1,numel(activeIDs)-1), ...
                'maxFE',maxFE-warmFE);
            if method=="AlwaysVND"
                search.enabled=true; search.mode='dynamic-vnd';
                search.localSearchFE=search.intensificationFE;
            elseif method=="FB-CMPSO"
                search.enabled=true; search.mode='boundary-vnd';
                search.localSearchFE=search.intensificationFE;
            end
            rng(algorithmSeed,'twister');
            maxIt=ceil((maxFE-warmFE)/nPop)+1;
            [solution,~,stats,history]=RoutingPSO(model,cache,maxIt,nPop, ...
                .90,.995,1.7,1.7,warmPositions,search); %#ok<ASGLU>
            assert(stats.functionEvaluations<=maxFE-warmFE,'FE超预算');
            for cp=checkpoints
                idx=find(warmFE+history.FE<=cp,1,'last'); if isempty(idx), continue; end
                actualFE=warmFE+history.FE(idx);
                firstFE=warmFE+stats.firstFeasibleEvaluation;
                if ~isfinite(firstFE)||firstFE>actualFE, firstFE=NaN; end
                switchFE=warmFE+stats.phaseSwitchFE;
                if ~isfinite(switchFE)||switchFE>actualFE, switchFE=NaN; end
                phaseGain=NaN;
                if isfinite(switchFE), phaseGain=stats.phaseSwitchCost-history.Cost(idx); end
                rows{end+1,1}={files(scenario),instance.scenarioSeed,algorithmSeed, ...
                    method,cp,actualFE,warmFE,history.RestorationFE(idx), ...
                    history.IntensificationFE(idx),warmTime+history.Time(idx), ...
                    history.Cost(idx),history.Distance(idx),history.Late(idx), ...
                    history.ObstacleViolation(idx),history.TerrainViolation(idx), ...
                    history.IsFeasible(idx),stats.initialBestFeasible, ...
                    stats.initialLate,stats.initialObstacleViolation,stats.initialTerrainViolation, ...
                    firstFE,switchFE,phaseGain,instance.referenceCost, ...
                    history.Cost(idx)<repair.Cost}; %#ok<AGROW>
            end
            if run==1 && any(files(scenario)==["addition_M1_09.mat","stress_Tight_01.mat"])
                demo.file=files(scenario); demo.model=model; demo.instance=instance;
                demo.previous=previous; demo.repair=repair;
                demo.method=method; demo.solution=solution; demo.history=history;
                save(fullfile(outDir,sprintf('route_demo_%s_%s.mat', ...
                    erase(char(files(scenario)),'.mat'),char(method))),'demo','-v7.3');
            end
        end
    end
    fprintf('%s: %d runs complete.\n',files(scenario),nRuns);
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'file','scenario_seed','algorithm_seed','method','checkpoint_fe','actual_fe', ...
    'warm_fe','restoration_fe','intensification_fe','time_seconds','fitness', ...
    'distance','total_late','obstacle_violation','terrain_violation','is_feasible', ...
    'initial_best_feasible','initial_total_late','initial_obstacle_violation', ...
    'initial_terrain_violation','first_feasible_fe','phase_switch_fe', ...
    'phase_gain','reference_cost','overtake_repair'});
legAudit=cell2table(vertcat(legRows{:}),'VariableNames',{ ...
    'file','cached_legs','detour_legs','unsafe_legs','minimum_clearance', ...
    'min_customer_altitude','max_customer_altitude'});
results.summary=summary; results.legAudit=legAudit; results.methods=methods;
results.files=files; results.checkpoints=checkpoints; results.nRuns=nRuns;
writetable(summary,fullfile(outDir,sprintf('%s_summary.csv',tag)));
writetable(legAudit,fullfile(outDir,sprintf('%s_leg_audit.csv',tag)));
save(fullfile(outDir,sprintf('%s_result.mat',tag)),'results','-v7.3');
end
