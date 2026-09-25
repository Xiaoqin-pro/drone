function RunAffectedRegionPrototype
%RUNAFFECTEDREGIONPROTOTYPE 比较Warm、Affected-region PSO及其局部搜索原型
scriptDir=fileparts(mfilename('fullpath'));
root=fileparts(scriptDir);
resultDir=fullfile(root,'results');
levels={'M1','M2','M4'};
rows=cell(0,1);
for li=1:numel(levels)
    for instanceID=1:2
        fileName=sprintf('addition_%s_%02d.mat',levels{li},instanceID);
        loaded=load(fullfile(resultDir,'addition_instances',fileName));
        instance=loaded.instance;
        model=instance.model; model.cfg.silent=true;
        cache=instance.cache; activeIDs=instance.activeIDs; event=instance.event;
        baseModel=CreateModel(); baseModel.windows(:,2)=baseModel.windows(:,2)+300; baseModel.cfg.silent=true;
        ids=baseModel.customerIDs(:)'; baseCache=BuildLegCache(baseModel,ids,baseModel.homePosition);
        rng(860000,'twister');
        [previousSolution,~,~]=RoutingPSO(baseModel,baseCache,100,40, ...
            0.90,0.995,1.7,1.7,[]);
        [affectedIDs,info]=DetectAffectedRegion(previousSolution.Detail.routeIDs, ...
            activeIDs,event.customerIDs,model,cache);
        seed=12000+100*li+instanceID;
        rng(seed,'twister');
        [warmPos,warmInfo]=BuildWarmStartPopulation(previousSolution, ...
            activeIDs,model,cache,30);
        available=1000-warmInfo.evaluations; maxIt=max(0,floor(available/30)-1);
        rng(seed,'twister');
        [warmSol,~,warmStats]=RoutingPSO(model,cache,maxIt,30, ...
            0.90,0.995,1.7,1.7,warmPos);
        rng(seed,'twister');
        [regionSol,~,regionStats]=AffectedRegionPSO(model,cache, ...
            previousSolution.Detail.routeIDs,affectedIDs,33,30, ...
            0.90,0.995,1.7,1.7,false,1000);
        rng(seed,'twister');
        [localSol,~,localStats]=AffectedRegionPSO(model,cache, ...
            previousSolution.Detail.routeIDs,affectedIDs,33,30, ...
            0.90,0.995,1.7,1.7,true,1000);
        names={'WarmStart','AffectedPSO','AffectedPSO_Local'};
        sols={warmSol,regionSol,localSol};
        fes=[warmStats.functionEvaluations+warmInfo.evaluations, ...
            regionStats.functionEvaluations,localStats.functionEvaluations];
        for k=1:numel(names)
            sol=sols{k};
            gap=(sol.Cost-instance.referenceCost)/max(abs(instance.referenceCost),eps);
            rows{end+1,1}={string(levels{li}),instanceID,string(names{k}), ...
                info.affectedCount,info.affectedRatio,fes(k),sol.Cost,gap, ...
                sol.Detail.distance,sol.Detail.totalLate,sol.Detail.isFeasible}; %#ok<AGROW>
        end
        fprintf('%s %d: affected=%d/%d\n',levels{li},instanceID, ...
            info.affectedCount,numel(activeIDs));
    end
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{ ...
    'level','instance','strategy','affected_count','affected_ratio', ...
    'total_fe','fitness','gap_to_witness','distance','late','is_feasible'});
writetable(summary,fullfile(resultDir,'affected_region_prototype.csv'));
save(fullfile(resultDir,'affected_region_prototype.mat'),'summary');
end
