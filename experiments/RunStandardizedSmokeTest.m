function RunStandardizedSmokeTest
%RUNSTANDARDIZEDSMOKETEST 标准来源V3的Repair/Warm/Restart最小验证
scriptDir=fileparts(mfilename('fullpath')); root=fileparts(scriptDir);
dataDir=fullfile(root,'data','standardized_dynamic'); files=dir(fullfile(dataDir,'*_dynamic.mat')); budgets=[500 1000]; rows=cell(0,1);
for f=1:numel(files)
    loaded=load(fullfile(dataDir,files(f).name)); instance=loaded.instance; model=instance.model; model.cfg.silent=true;
    preModel=model; preModel.startTime=0; preModel.depot=model.homePosition; preIDs=unique(instance.preEventRoute,'stable'); preCache=BuildLegCache(preModel,preIDs,model.homePosition);
    [preCost,preDetail]=EvaluateSchedule(instance.preEventRoute,preModel,preCache); preSolution.Cost=preCost; preSolution.Detail=preDetail; preSolution.Route=preDetail.routeIDs;
    postModel=model; postModel.startTime=instance.event.time; postModel.depot=instance.preEventState.position; postIDs=unique([instance.activeIDs(:)',instance.hiddenIDs(:)'],'stable'); postCache=BuildLegCache(postModel,postIDs,instance.preEventState.position); event=instance.event;
    for b=budgets
        for strategy={'Restart','WarmStart','Repair'}
            name=strategy{1}; warmFE=0; warmTime=0;
            if strcmp(name,'Repair')
                t=tic; [sol,ri]=LocalRepair(preSolution,postIDs,postModel,postCache,'add',event.customerIDs); routeTime=toc(t); routeFE=ri.functionEvaluations;
            else
                if strcmp(name,'WarmStart'), [pos,wi]=BuildWarmStartPopulation(preSolution,postIDs,postModel,postCache,30); warmFE=wi.evaluations; else, pos=[]; end
                maxIt=max(0,floor((b-warmFE)/30)-1); rng(30000+b,'twister'); t=tic; [sol,~,st]=RoutingPSO(postModel,postCache,maxIt,30,.9,.995,1.7,1.7,pos); routeTime=toc(t); routeFE=st.functionEvaluations;
            end
            rows{end+1,1}={string(instance.sourceBenchmark),string(name),b,warmFE,routeFE,warmFE+routeFE,sol.Cost,sol.Detail.distance,sol.Detail.totalLate,sol.Detail.isFeasible}; %#ok<AGROW>
        end
    end
end
summary=cell2table(vertcat(rows{:}),'VariableNames',{'source','strategy','budget','warm_start_fe','routing_fe','total_fe','fitness','distance','late','is_feasible'}); writetable(summary,fullfile(root,'results','standardized_smoke_summary.csv')); disp(summary);
end
